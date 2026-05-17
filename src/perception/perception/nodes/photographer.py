#!/usr/bin/env python3

import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

from typing import Dict, List, Optional
from dataclasses import dataclass, field

import numpy as np


DEFAULT_OUTPUT_DIR = "real/wet_data_test"
DEFAULT_CAPTURE_PERIOD_S = 0.8
DEFAULT_IMAGE_EXT = "jpg"


@dataclass
class Camera:
    name: str
    rgb: Optional[np.ndarray] = None
    rgb_stamp_ns: Optional[int] = None
    last_saved_stamp_ns: Optional[int] = None
    saved_count: int = 0
    subscribers: List[Subscription] = field(default_factory=list)


class PhotographerNode(Node):
    def __init__(self, camera_names: List[str]):
        super().__init__("photographer")
        self.bridge = CvBridge()

        self.declare_parameter("output_dir", DEFAULT_OUTPUT_DIR)
        self.declare_parameter("capture_period_s", DEFAULT_CAPTURE_PERIOD_S)
        self.declare_parameter("image_ext", DEFAULT_IMAGE_EXT)

        self.output_dir: str = (
            self.get_parameter("output_dir").get_parameter_value().string_value
            or DEFAULT_OUTPUT_DIR
        )
        self.capture_period_s: float = (
            self.get_parameter("capture_period_s").get_parameter_value().double_value
            or DEFAULT_CAPTURE_PERIOD_S
        )
        self.image_ext: str = (
            self.get_parameter("image_ext").get_parameter_value().string_value
            or DEFAULT_IMAGE_EXT
        ).lstrip(".")

        # Resolve output dir to an absolute path so images always land in the same
        # place regardless of where the node is launched from.
        if not os.path.isabs(self.output_dir):
            self.output_dir = os.path.abspath(self.output_dir)

        self.cameras: Dict[str, Camera] = {}
        for name in camera_names:
            self._setup_camera(name)

        self.save_timer = self.create_timer(self.capture_period_s, self.save_latest)

        self.get_logger().info(
            f"Photographer ready: saving every {self.capture_period_s:.3f}s "
            f"to '{self.output_dir}' for cameras {list(self.cameras.keys())}"
        )

    def _setup_camera(self, name: str) -> None:
        camera = Camera(name=name)
        rgb_topic = f"/camera/{name}/rgb"

        camera.subscribers.append(
            self.create_subscription(
                Image, rgb_topic, self._rgb_callback(camera), 10
            )
        )

        camera_dir = os.path.join(self.output_dir, name)
        os.makedirs(camera_dir, exist_ok=True)

        self.cameras[name] = camera
        self.get_logger().info(f"Subscribed to {rgb_topic} (saving to {camera_dir})")

    def _rgb_callback(self, camera: Camera):
        def callback(msg: Image):
            try:
                camera.rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                stamp = msg.header.stamp
                camera.rgb_stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            except Exception as e:
                self.get_logger().error(
                    f"Failed to decode RGB image from {camera.name}: {e}"
                )

        return callback

    def save_latest(self) -> None:
        for camera in self.cameras.values():
            if camera.rgb is None:
                continue

            # Skip frames we have already saved (avoids dumping duplicates when a
            # camera stream stalls).
            if (
                camera.rgb_stamp_ns is not None
                and camera.last_saved_stamp_ns == camera.rgb_stamp_ns
            ):
                continue

            stamp_ns = camera.rgb_stamp_ns or time.time_ns()
            filename = f"{stamp_ns}.{self.image_ext}"
            path = os.path.join(self.output_dir, camera.name, filename)

            try:
                ok = cv2.imwrite(path, camera.rgb)
                if not ok:
                    raise IOError(f"cv2.imwrite returned False for {path}")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to save frame for {camera.name} to {path}: {e}"
                )
                continue

            camera.last_saved_stamp_ns = camera.rgb_stamp_ns
            camera.saved_count += 1
            self.get_logger().debug(
                f"Saved {camera.name} frame #{camera.saved_count} -> {path}"
            )


def _parse_camera_names(argv: List[str]) -> List[str]:
    if len(argv) <= 1:
        return []
    # Support both `photographer cam_a cam_b` and `photographer cam_a,cam_b`.
    raw = " ".join(argv[1:])
    parts = [p.strip() for chunk in raw.split(",") for p in chunk.split()]
    return [p for p in parts if p]


def main(args=None):
    rclpy.init(args=args)

    argv = remove_ros_args(args if args is not None else None)
    camera_names = _parse_camera_names(argv)

    if not camera_names:
        print(
            "Usage: ros2 run perception photographer <camera_name> [<camera_name> ...]\n"
            "       ros2 run perception photographer cam_a,cam_b"
        )
        rclpy.shutdown()
        return

    node = PhotographerNode(camera_names)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
