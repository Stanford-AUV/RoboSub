#!/usr/bin/env python3
import threading
import time
import json
from dataclasses import dataclass
from typing import Optional, Tuple, Dict, List

import rclpy
from rclpy.node import Node
from rclpy import Parameter

import numpy as np
import cv2
import depthai as dai

from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose


@dataclass
class CamCfg:
    name: str                 # "forward_cam" or "bottom_cam"
    mx_id: str                # device MXID
    nn_blob: str              # per-camera blob path
    nn_json: str              # per-camera json path
    view_detections: bool     # draw local windows
    pub_topic: str            # e.g. "forward_cam/detections3d"


class ObjectsLocalizerBoth(Node):
    def __init__(self):
        super().__init__("objects_localizer_both")

        #hmm i dont like how we have oak1 oak2 in one file and then raw mxids hanging out in a py file, maybe constants abstractions
        # ---------- Device IDs ----------
        self.declare_parameter("forward.mx_id", "19443010B17E1C1300")
        self.declare_parameter("bottom.mx_id",  "1944301021531E1300")

        # ---------- Global default NN (used if per-camera not provided) ----------
        self.declare_parameter("nn.blob",  "new_arvp_front_openvino_2022.1_6shave.blob")
        self.declare_parameter("nn.config","new_arvp_front.json")

        # ---------- Per-camera NN overrides ----------
        # If these are set, they override the global defaults for that camera.
        self.declare_parameter("forward.nn.blob", "new_arvp_front_openvino_2022.1_6shave.blob")
        self.declare_parameter("forward.nn.config", "new_arvp_front.json")
        self.declare_parameter("bottom.nn.blob", "")
        self.declare_parameter("bottom.nn.config", "")

        # Show OpenCV windows for debugging
        self.declare_parameter("view_detections", False)

        # Pull params
        f_mx = self.get_parameter("forward.mx_id").get_parameter_value().string_value
        b_mx = self.get_parameter("bottom.mx_id").get_parameter_value().string_value
        default_blob  = self.get_parameter("nn.blob").get_parameter_value().string_value
        default_json  = self.get_parameter("nn.config").get_parameter_value().string_value
        view_det = self.get_parameter("view_detections").get_parameter_value().bool_value

        f_blob = self._pick(self.get_parameter("forward.nn.blob").get_parameter_value().string_value, default_blob)
        f_json = self._pick(self.get_parameter("forward.nn.config").get_parameter_value().string_value, default_json)
        b_blob = self._pick(self.get_parameter("bottom.nn.blob").get_parameter_value().string_value, default_blob)
        b_json = self._pick(self.get_parameter("bottom.nn.config").get_parameter_value().string_value, default_json)

        # Validate blobs: must be .blob (DepthAI cannot load .pt/.pth directly)
        for cam_name, blob in [("forward", f_blob), ("bottom", b_blob)]:
            if not blob.lower().endswith(".blob"):
                self.get_logger().error(
                    f"[{cam_name}] Model path does not look like a compiled DepthAI blob: {blob}\n"
                    "DepthAI cannot load .pt/.pth/.pts directly. Please convert your PyTorch model "
                    "to a .blob (e.g., via Luxonis tools) and set <camera>.nn.blob to that file."
                )
                raise RuntimeError(f"{cam_name} NN blob must be a .blob")

        # Camera configs (per-camera models)
        self.cams: List[CamCfg] = [
            CamCfg(name="forward_cam", mx_id=f_mx, nn_blob=f_blob, nn_json=f_json,
                   view_detections=view_det, pub_topic="forward_cam/detections3d"),
            CamCfg(name="bottom_cam",  mx_id=b_mx, nn_blob=b_blob, nn_json=b_json,
                   view_detections=view_det, pub_topic="bottom_cam/detections3d"),
        ]

        # Publishers
        self.pubs: Dict[str, rclpy.publisher.Publisher] = {
            c.name: self.create_publisher(Detection3DArray, c.pub_topic, 10) for c in self.cams
        }

        # Threads
        self.stop_flag = False
        self.threads: List[threading.Thread] = []
        for cfg in self.cams:
            t = threading.Thread(target=self._run_device_loop, args=(cfg,), daemon=True)
            t.start()
            self.threads.append(t)
            self.get_logger().info(f"Started device thread for {cfg.name} (MXID {cfg.mx_id}) "
                                   f"with model '{cfg.nn_blob}' / config '{cfg.nn_json}'")

        self.get_logger().info("ObjectsLocalizerBoth running (two cameras, per-camera models).")

    @staticmethod
    def _pick(primary: str, fallback: str) -> str:
        return primary if primary else fallback

    # --------- DepthAI per-device runner ----------
    def _run_device_loop(self, cfg: CamCfg):
        # Load NN config (per camera)
        try:
            with open(cfg.nn_json, "r") as f:
                conf = json.load(f)
        except Exception as e:
            self.get_logger().error(f"[{cfg.name}] Failed to read NN config '{cfg.nn_json}': {e}")
            return

        nn_cfg = conf.get("nn_config", {})
        if "input_size" not in nn_cfg:
            self.get_logger().error(f"[{cfg.name}] nn_config.input_size missing in {cfg.nn_json}")
            return
        try:
            W, H = tuple(map(int, nn_cfg["input_size"].split("x")))
        except Exception as e:
            self.get_logger().error(f"[{cfg.name}] Bad input_size format in {cfg.nn_json}: {e}")
            return

        meta = nn_cfg.get("NN_specific_metadata", {})
        classes = int(meta.get("classes", 0))
        coordinates = int(meta.get("coordinates", 0))
        anchors = meta.get("anchors", [])
        anchorMasks = meta.get("anchor_masks", {})
        iouThreshold = float(meta.get("iou_threshold", 0.5))
        confidenceThreshold = float(meta.get("confidence_threshold", 0.5))
        labels = conf.get("mappings", {}).get("labels", [])

        # Build pipeline
        pipeline = dai.Pipeline()

        #too much yap. can prob helper it out/ helper class it out
        camRgb = pipeline.create(dai.node.ColorCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)
        yolo = pipeline.create(dai.node.YoloSpatialDetectionNetwork)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutNet = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName(f"{cfg.name}_rgb")
        xoutDepth.setStreamName(f"{cfg.name}_depth")
        xoutNN.setStreamName(f"{cfg.name}_detections")
        xoutNet.setStreamName(f"{cfg.name}_nnNetwork")

        # Camera properties
        camRgb.setPreviewSize(W, H)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        # Depth aligned to RGB (CAM_A)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())
        stereo.setSubpixel(True)

        # YOLO spatial (per-camera blob/config)
        yolo.setConfidenceThreshold(confidenceThreshold)
        yolo.setNumClasses(classes)
        yolo.setCoordinateSize(coordinates)
        yolo.setAnchors(anchors)
        yolo.setAnchorMasks(anchorMasks)
        yolo.setIouThreshold(iouThreshold)
        yolo.setBlobPath(cfg.nn_blob)
        yolo.setNumInferenceThreads(2)
        yolo.input.setBlocking(False)

        # Linking
        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        camRgb.preview.link(yolo.input)
        yolo.passthrough.link(xoutRgb.input)
        yolo.out.link(xoutNN.input)
        stereo.depth.link(yolo.inputDepth)
        yolo.passthroughDepth.link(xoutDepth.input)
        yolo.outNetwork.link(xoutNet.input)

        # Open the specific device
        try:
            dinfo = dai.DeviceInfo(cfg.mx_id)
            with dai.Device(pipeline, dinfo) as device:
                calib = device.readCalibration()
                K = np.array(calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, W, H))
                Kinv = np.linalg.inv(K)

                # Queues
                q_rgb = device.getOutputQueue(name=f"{cfg.name}_rgb", maxSize=4, blocking=False)
                q_det = device.getOutputQueue(name=f"{cfg.name}_detections", maxSize=4, blocking=False)
                q_depth = device.getOutputQueue(name=f"{cfg.name}_depth", maxSize=4, blocking=False)
                q_net = device.getOutputQueue(name=f"{cfg.name}_nnNetwork", maxSize=4, blocking=False)

                # self.get_logger().info(f"q_det is {q_det}")

                printed_layers = False
                while not self.stop_flag and rclpy.ok():
                    in_rgb = q_rgb.tryGet()
                    in_det = q_det.tryGet()
                    in_depth = q_depth.tryGet()
                    in_net = q_net.tryGet()

                    if in_rgb:
                        frame = in_rgb.getCvFrame()
                        self.get_logger().info(
                            f"[{cfg.name}] Got RGB frame: shape={frame.shape}, dtype={frame.dtype}"
                        )
                    else:
                        # self.get_logger().warn(f"[{cfg.name}] No RGB frame in queue.")
                        pass


                    if in_net and not printed_layers:
                        names = " ".join(in_net.getAllLayerNames())
                        self.get_logger().info(f"[{cfg.name}] NN Layers: {names}")
                        printed_layers = True

                    if not (in_rgb and in_det and in_depth):
                        time.sleep(0.001)
                        continue

                    frame = in_rgb.getCvFrame()
                    depth = in_depth.getFrame()  # mm
                    dets = in_det.detections

                    # Build ROS message
                    msg = Detection3DArray()
                    # Optional header:
                    # msg.header.frame_id = f"{cfg.name}_optical_frame"
                    # msg.header.stamp = self.get_clock().now().to_msg()

                    h, w = frame.shape[:2]

                    if cfg.view_detections:
                        depth_ds = depth[::4]
                        if np.all(depth_ds == 0):
                            min_d = 0
                        else:
                            min_d = np.percentile(depth_ds[depth_ds != 0], 1)
                        max_d = np.percentile(depth_ds, 99)
                        depth_color = np.interp(depth, (min_d, max_d), (0, 255)).astype(np.uint8)
                        depth_color = cv2.applyColorMap(depth_color, cv2.COLORMAP_HOT)

                    for d in dets:
                        lbl = labels[d.label] if (0 <= d.label < len(labels)) else str(d.label)
                        score = d.confidence

                        roi = d.boundingBoxMapping.roi.denormalize(depth.shape[1], depth.shape[0])
                        tl, br = roi.topLeft(), roi.bottomRight()
                        xmin, ymin = int(tl.x), int(tl.y)
                        xmax, ymax = int(br.x), int(br.y)

                        sc = d.spatialCoordinates  # mm

                        det3d = Detection3D()
                        ohp = ObjectHypothesisWithPose()
                        ohp.hypothesis.class_id = lbl
                        ohp.hypothesis.score = float(score)
                        det3d.results.append(ohp)

                        center = Pose()
                        center.position.x = sc.x / 1000.0
                        center.position.y = sc.y / 1000.0
                        center.position.z = sc.z / 1000.0
                        det3d.bbox.center = center

                        if sc.z > 0:
                            x1 = int(d.xmin * w); x2 = int(d.xmax * w)
                            y1 = int(d.ymin * h); y2 = int(d.ymax * h)
                            tl_pix = np.array([x1, y1, 1.0])
                            br_pix = np.array([x2, y2, 1.0])
                            z_m = sc.z / 1000.0
                            tl_cam = (z_m * (Kinv @ tl_pix))
                            br_cam = (z_m * (Kinv @ br_pix))
                            det3d.bbox.size.x = float(abs(br_cam[0] - tl_cam[0]))
                            det3d.bbox.size.y = float(abs(br_cam[1] - tl_cam[1]))
                            det3d.bbox.size.z = 0.0
                        else:
                            det3d.bbox.size.x = 0.0
                            det3d.bbox.size.y = 0.0
                            det3d.bbox.size.z = 0.0

                        msg.detections.append(det3d)

                        if cfg.view_detections:
                            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (255,255,255), 1)
                            x1 = int(d.xmin * w); x2 = int(d.xmax * w)
                            y1 = int(d.ymin * h); y2 = int(d.ymax * h)
                            cv2.rectangle(frame, (x1,y1),(x2,y2),(0,255,0),1)
                            cv2.putText(frame, f"{lbl} {score:.2f}", (x1+6,y1+18),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)

                    self.pubs[cfg.name].publish(msg)

                    if cfg.view_detections:
                        cv2.imshow(f"{cfg.name}_rgb", frame)
                        cv2.imshow(f"{cfg.name}_depth", depth_color)
                        cv2.waitKey(1)

        except Exception as e:
            self.get_logger().warn(f"[{cfg.name}] Could not open device {cfg.mx_id}: {e}")

    def destroy_node(self):
        self.stop_flag = True
        time.sleep(0.05)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ObjectsLocalizerBoth()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
