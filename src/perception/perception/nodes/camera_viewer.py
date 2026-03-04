#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

from typing import Optional, List
from dataclasses import dataclass, field


@dataclass
class Camera:
    name: str
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    subscribers: List[Subscription] = field(default_factory=list)


class CameraViewerNode(Node):
    def __init__(self):
        super().__init__("camera_viewer_node")
        self.bridge = CvBridge()

        # Initialize image holders for both cameras
        self.oak1_rgb = None
        self.oak1_depth = None
        self.oak2_rgb = None
        self.oak2_depth = None

        self.cameras = [Camera(name="forward_cam"), Camera(name="bottom_cam")]

        # Subscribe to topics
        rgb_topic_suffix = "rgb/image_rect/compressed"
        depth_topic_suffix = "stereo/image_raw"
        for camera in self.cameras:
            rgb_topic = f"/{camera.name}/{rgb_topic_suffix}"
            depth_topic = f"/{camera.name}/{depth_topic_suffix}"
            camera.subscribers.append(
                self.create_subscription(
                    CompressedImage, rgb_topic, self.image_callback(camera, "rgb"), 10
                )
            )
            camera.subscribers.append(
                self.create_subscription(
                    Image, depth_topic, self.image_callback(camera, "depth"), 10
                )
            )
            self.get_logger().info(f"Subscribed to {rgb_topic} and {depth_topic}")

        # Window name
        self.window_name = "Dual Camera View"

        # Create a timer for displaying the view
        FPS = 10
        self.display_timer = self.create_timer(1.0 / FPS, self.display_view)  # ~30fps

    def image_callback(self, camera: Camera, image_type: str):
        """Callback for compressed image frames."""

        def callback(msg):
            try:
                if image_type == "rgb":
                    img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                else:
                    img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
                
                DOWNSAMPLE_FACTOR = 4
                img = cv2.resize(img, dsize=(img.shape[1]//DOWNSAMPLE_FACTOR, img.shape[0]//DOWNSAMPLE_FACTOR))

                if image_type == "rgb":
                    camera.rgb = img
                else:
                    camera.depth = img
            except Exception as e:
                self.get_logger().error(
                    f"Failed to process {image_type} image from {camera.name}: {e}"
                )
            self.get_logger().info(f"Processed {image_type} image from {camera.name}")

        return callback

    def process_camera_view(self, rgb_img, depth_img, camera_name):
        """Process a single camera's view (RGB + Depth side by side)."""
        if rgb_img is not None and depth_img is not None:
            # Convert depth to color map
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET
            )

            # Resize depth to match RGB
            if rgb_img.shape[:2] != depth_colormap.shape[:2]:
                depth_colormap = cv2.resize(
                    depth_colormap, (rgb_img.shape[1], rgb_img.shape[0])
                )

            # Combine RGB and depth side by side
            combined = cv2.hconcat([rgb_img, depth_colormap])

            # Add camera name label
            cv2.putText(
                combined,
                camera_name.upper(),
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                combined, "RGB", (30, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2
            )
            cv2.putText(
                combined,
                "Depth",
                (rgb_img.shape[1] + 30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )

            return combined
        return None

    def display_view(self):
        """Display both cameras' views stacked vertically."""
        # Stack vertically all views
        display_img = None
        for camera in self.cameras:
            view = self.process_camera_view(camera.rgb, camera.depth, camera.name)
            if view is not None:
                if display_img is not None:
                    display_img = cv2.vconcat([display_img, view])
                else:
                    display_img = view

        if display_img is not None:
            cv2.imshow(self.window_name, display_img)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
