#!/usr/bin/env python3
"""Publishes AlignedDepthImage (Option A) from either OAK or RealSense backend."""

import os
import time
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from builtin_interfaces.msg import Time as BuiltinTime
from msgs.msg import AlignedDepthImage

from .aligned_depth_backends import OakAlignedBackend, RealsenseAlignedBackend


class AlignedDepthPublisherNode(Node):
    def __init__(self, yaml_path: str):
        super().__init__("aligned_depth_publisher")

        self.declare_parameter("camera_type", "oak")
        self.declare_parameter("camera_key", "oak_0")

        camera_type = self.get_parameter("camera_type").get_parameter_value().string_value
        camera_key = self.get_parameter("camera_key").get_parameter_value().string_value

        self._bridge = CvBridge()
        self._camera_key = camera_key
        self._camera_type = camera_type.lower()

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Cameras yaml not found: {yaml_path}")
        with open(yaml_path, "r") as f:
            import yaml
            data = yaml.safe_load(f)

        cam_data = {}
        for key, value in data.items():
            if self._camera_type == "oak" and "oak" in key:
                cam_data[key] = value
            elif self._camera_type == "realsense" and "realsense" in key:
                cam_data[key] = value

        if camera_key not in cam_data:
            raise KeyError(
                f"Camera key '{camera_key}' not in yaml (found: {list(cam_data.keys())})"
            )
        cam_cfg = cam_data[camera_key]

        if self._camera_type == "oak":
            self._backend = OakAlignedBackend(camera_key, cam_cfg, self.get_logger())
        elif self._camera_type == "realsense":
            self._backend = RealsenseAlignedBackend(camera_key, cam_cfg, self.get_logger())
        else:
            raise ValueError(f"Unknown camera_type: {camera_type} (use 'oak' or 'realsense')")

        self._pub = self.create_publisher(
            AlignedDepthImage,
            f"/camera/{camera_key}/aligned",
            10,
        )
        self._timer_count = 0
        self._get_frame_none_count = 0
        self.get_logger().info(
            f"Publishing AlignedDepthImage on /camera/{camera_key}/aligned ({camera_type})"
        )
        self.create_timer(1.0 / 30.0, self._publish_frame)

    def _publish_frame(self):
        t0 = time.perf_counter()
        stamp = self.get_clock().now().to_msg()
        frame = self._backend.get_frame(stamp)
        t1 = time.perf_counter()
        if frame is None:
            self._get_frame_none_count += 1
            self._timer_count += 1
            if self._timer_count % 30 == 0:
                self.get_logger().info(
                    f"aligned_depth_publisher: get_frame returned None {self._get_frame_none_count}/30 ticks (backend not ready)"
                )
                self._get_frame_none_count = 0
            return

        rgb_msg = self._bridge.cv2_to_imgmsg(frame.rgb, "bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = frame.camera_info.header.frame_id

        depth_msg = self._bridge.cv2_to_imgmsg(frame.depth, "16UC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = frame.camera_info.header.frame_id

        frame.camera_info.header.stamp = stamp

        out = AlignedDepthImage()
        out.rgb = rgb_msg
        out.depth = depth_msg
        out.camera_info = frame.camera_info
        out.hardware_stamp = BuiltinTime(
            sec=frame.hardware_stamp_sec,
            nanosec=frame.hardware_stamp_nanosec,
        )
        self._pub.publish(out)
        t2 = time.perf_counter()

        self._timer_count += 1
        get_frame_ms = (t1 - t0) * 1000
        build_publish_ms = (t2 - t1) * 1000
        if self._timer_count % 30 == 0:
            self.get_logger().info(
                f"aligned_depth_publisher timing (tick #{self._timer_count}): "
                f"get_frame={get_frame_ms:.1f}ms build_publish={build_publish_ms:.1f}ms (None in last 30: {self._get_frame_none_count})"
            )
            self._get_frame_none_count = 0

    def destroy_node(self):
        self._backend.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    script_dir = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(script_dir, "..", "cameras.yaml")
    yaml_path = os.path.abspath(yaml_path)

    node = AlignedDepthPublisherNode(yaml_path)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
