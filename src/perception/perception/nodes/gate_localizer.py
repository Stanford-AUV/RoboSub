#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose
import message_filters
from cv_bridge import CvBridge
import numpy as np
import cv2
import time

def backproject_bbox_size(px_w, px_h, z_m, fx, fy):
    return (px_w / fx) * z_m, (px_h / fy) * z_m

class GateLocalizer(Node):
    def __init__(self):
        super().__init__("gate_localizer")

        # Params
        self.declare_parameter("view_detections", False)
        self.declare_parameter("min_area", 800)
        self.declare_parameter("min_aspect_ratio", 2.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")

        self.declare_parameter("rgb_topic", "rgb/image_raw")
        self.declare_parameter("depth_topic", "stereo/depth")
        self.declare_parameter("camera_info_topic", "rgb/camera_info")

        self.view = self.get_parameter("view_detections").get_parameter_value().bool_value
        self.min_area = int(self.get_parameter("min_area").value)
        self.min_ar = float(self.get_parameter("min_aspect_ratio").value)
        self.default_frame = self.get_parameter("frame_id").value

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value

        # HSV thresholds
        self.red1 = (np.array([0, 100, 70]),   np.array([10, 255, 255]))
        self.red2 = (np.array([170, 100, 70]), np.array([180, 255, 255]))
        self.white = (np.array([0, 0, 200]),   np.array([180, 40, 255]))

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Detection3DArray, "detections3d", 10)

        # Sensor QoS
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        sensor_qos.history = QoSHistoryPolicy.KEEP_LAST

        # Subscriptions (synced)
        rgb_sub   = message_filters.Subscriber(self, Image,      self.rgb_topic,   qos_profile=sensor_qos)
        depth_sub = message_filters.Subscriber(self, Image,      self.depth_topic, qos_profile=sensor_qos)
        info_sub  = message_filters.Subscriber(self, CameraInfo, self.info_topic,  qos_profile=sensor_qos)

        sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub, info_sub], queue_size=10, slop=0.15
        )
        sync.registerCallback(self.callback)

        self.get_logger().info(
            f"gate_localizer subscribed to:\n"
            f"  RGB:   {self.rgb_topic}\n"
            f"  Depth: {self.depth_topic}\n"
            f"  Info:  {self.info_topic}"
        )

    # ---------- Vision helpers ----------
    def _mask_color(self, hsv):
        red_mask = cv2.inRange(hsv, *self.red1) | cv2.inRange(hsv, *self.red2)
        white_mask = cv2.inRange(hsv, *self.white)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask   = cv2.morphologyEx(red_mask,   cv2.MORPH_OPEN, k, iterations=1)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, k, iterations=1)
        return {"red_tube": red_mask, "white_tube": white_mask}
        # why are we using an ellipse??

    def _find_tube_bboxes(self, mask):
        #we don't like the min area, seems like it will only work at a very close range. something more dynamic.
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            ar1 = (h / max(w, 1.0))
            ar2 = (w / max(h, 1.0))
            if ar1 < self.min_ar and ar2 < self.min_ar:
                continue
            roi = mask[y:y+h, x:x+w]

            # this is so sus. we should change this. maybe with a cnn
            fill = float(np.count_nonzero(roi)) / (w * h + 1e-6)
            boxes.append((x, y, w, h, float(np.clip(fill, 0.0, 1.0))))
        return boxes

    # ---------- Synced callback ----------
    def callback(self, rgb_msg: Image, depth_msg: Image, info_msg: CameraInfo):
        # Convert images
        frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg)  # 16UC1 in mm or 32FC1 in m

        # Intrinsics
        K = np.array(info_msg.k, dtype=np.float32).reshape(3, 3)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = self._mask_color(hsv)

        msg = Detection3DArray()
        msg.header.stamp = rgb_msg.header.stamp
        msg.header.frame_id = info_msg.header.frame_id or self.default_frame

        debug_vis = frame.copy() if self.view else None

        # Normalize depth to meters
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) / 1000.0
        else:
            depth_m = depth.astype(np.float32)

        for label, mask in masks.items():
            for (x, y, w, h, score) in self._find_tube_bboxes(mask):
                roi_depth = depth_m[y:y+h, x:x+w]
                z_vals = roi_depth[roi_depth > 0]
                if z_vals.size == 0:
                    continue
                z_m = float(np.median(z_vals))

                # u v is camera coords, X Y is world
                u = x + w / 2.0
                v = y + h / 2.0
                # we need the Rt part
                X = (u - cx) / fx * z_m
                Y = (v - cy) / fy * z_m

                size_x_m, size_y_m = backproject_bbox_size(w, h, z_m, fx, fy)

                det = Detection3D()
                hypo = ObjectHypothesisWithPose()
                hypo.hypothesis.class_id = label
                hypo.hypothesis.score = float(score)
                det.results.append(hypo)

                center = Pose()
                center.position.x = X
                center.position.y = Y
                center.position.z = z_m
                det.bbox.center = center
                det.bbox.size.x = float(max(size_x_m, 1e-3))
                det.bbox.size.y = float(max(size_y_m, 1e-3))
                det.bbox.size.z = 0.0

                msg.detections.append(det)

                if self.view:
                    color = (0, 0, 255) if "red" in label else (255, 255, 255)
                    cv2.rectangle(debug_vis, (x, y), (x+w, y+h), color, 2)
                    cv2.putText(debug_vis, f"{label} {score:.2f}",
                                (x, max(0, y-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        self.pub.publish(msg)

        if self.view:
            red_vis = cv2.cvtColor(masks["red_tube"], cv2.COLOR_GRAY2BGR)
            white_vis = cv2.cvtColor(masks["white_tube"], cv2.COLOR_GRAY2BGR)
            h1 = debug_vis
            h2 = cv2.resize(red_vis, (h1.shape[1], h1.shape[0]))
            h3 = cv2.resize(white_vis, (h1.shape[1], h1.shape[0]))
            top = np.hstack([h1, h2, h3])
            cv2.imshow("gate_localizer (rgb | red_mask | white_mask)", top)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GateLocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
