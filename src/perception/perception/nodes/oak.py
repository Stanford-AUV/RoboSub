#!/usr/bin/env python3
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os

resolution_map = {
    "400P": dai.MonoCameraProperties.SensorResolution.THE_400_P,
    "480P": dai.MonoCameraProperties.SensorResolution.THE_480_P,
    "720P": dai.MonoCameraProperties.SensorResolution.THE_720_P,
    "800P": dai.MonoCameraProperties.SensorResolution.THE_800_P,
    "1080P": dai.MonoCameraProperties.SensorResolution.THE_1200_P,
}

preset_map = {
    "FAST_ACCURACY": dai.node.StereoDepth.PresetMode.FAST_ACCURACY,
    "FAST_DENSITY": dai.node.StereoDepth.PresetMode.FAST_DENSITY,
}


def load_cameras_yaml(path):
    if not os.path.exists(path):
        raise FileNotFoundError("Noooooo! No yaml path exists :(")

    with open(path, "r") as f:
        data = yaml.safe_load(f)

    return data


def get_oak_cam_data(data):
    oak_data = {}
    for key, value in data.items():
        if "oak" in key:
            oak_data[key] = value
    return oak_data


def config_rgb_image(pipeline, key, cam_cfg):
    rgb_cfg = cam_cfg.get("rgb")

    width = rgb_cfg.get("rgb_width", 640)
    height = rgb_cfg.get("rgb_height", 480)
    fps = rgb_cfg.get("rgb_fps", 30)

    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setPreviewSize(width, height)
    cam_rgb.setFps(fps)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

    video_queue = cam_rgb.video.createOutputQueue()
    return video_queue


def config_stereo_image(pipeline, key, cam_cfg):
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)

    stereo_cfg = cam_cfg.get("stereo")

    stereo.setDefaultProfilePreset(preset_map[stereo_cfg.get("i_depth_preset")])
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)

    stereo.setExtendedDisparity(stereo_cfg.get("i_extended_disp", False))

    mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
    mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

    mono_left.setResolution(resolution_map[stereo_cfg["left"]["i_resolution"]])
    mono_right.setResolution(resolution_map[stereo_cfg["right"]["i_resolution"]])

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    depth_queue = stereo.depth.createOutputQueue()
    return depth_queue


class OakNode(Node):
    def __init__(self, _camera_info):
        super().__init__("oak_node")

        self.bridge = CvBridge()
        self.rgb_pubs = {}
        self.depth_pubs = {}
        self.rgbd_pubs = {}

        self.devices = {}  # holds dai.Device objects
        self.queues = {}  # holds output queues

        for key, cam_cfg in _camera_info.items():
            cam_cfg = cam_cfg.get("ros__parameters")

            self.get_logger().info(
                f"Starting OAK camera: {key} (mxid={cam_cfg['camera']['i_mx_id']})"
            )

            rgb_pub = self.create_publisher(Image, f"/camera/{key}/rgb", 10)
            self.rgb_pubs[key] = rgb_pub

            depth_pub = self.create_publisher(Image, f"/camera/{key}/depth", 10)
            self.depth_pubs[key] = depth_pub

            rgbd_pub = self.create_publisher(Image, f"/camera/{key}/rgbd", 10)

            self.rgbd_pubs[key] = rgbd_pub

            device = dai.Device(cam_cfg["camera"]["i_mx_id"])
            pipeline = dai.Pipeline(device)

            rgb = config_rgb_image(pipeline, key, cam_cfg)
            depth = config_stereo_image(pipeline, key, cam_cfg)

            self.devices[key] = pipeline
            self.queues[key] = {
                "rgb": rgb,
                "depth": depth,
            }

            pipeline.start()

        self.create_timer(0.06, self.publish_frames)  # ~30 FPS

    def publish_frames(self):
        for key in self.devices.keys():
            rgb_frame = self.queues[key]["rgb"].tryGet()
            depth_frame = self.queues[key]["depth"].tryGet()

            if rgb_frame is not None:
                rgb = rgb_frame.getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
                self.rgb_pubs[key].publish(msg)

            if depth_frame is not None:
                depth = depth_frame.getFrame()
                msg = self.bridge.cv2_to_imgmsg(depth, "mono16")
                self.depth_pubs[key].publish(msg)

            # ---- fuse RGB + depth ---- #
            if rgb_frame is not None and depth_frame is not None:
                rgb = rgb_frame.getCvFrame()
                depth = depth_frame.getFrame()  # uint16

                # Normalize and colorize depth
                depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX)
                depth_norm = depth_norm.astype(np.uint8)
                depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

                # Resize depth to match RGB
                depth_color = cv2.resize(depth_color, (rgb.shape[1], rgb.shape[0]))

                # Fuse RGB + depth
                rgbd = cv2.addWeighted(rgb, 0.7, depth_color, 0.3, 0)

                # Publish
                msg = self.bridge.cv2_to_imgmsg(rgbd, "bgr8")
                self.rgbd_pubs[key].publish(msg)


def main(args=None):
    rclpy.init(args=args)

    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(SCRIPT_DIR, "..", "cameras.yaml")
    yaml_path = os.path.abspath(yaml_path)

    camera_info = load_cameras_yaml(yaml_path)
    oak_info = get_oak_cam_data(camera_info)

    node = OakNode(oak_info)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
