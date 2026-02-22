#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.utilities import remove_ros_args

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import yaml

from typing import Optional, List
from dataclasses import dataclass, field

from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass
class Camera:
    name: str
    rgb: Optional[np.ndarray] = None
    depth: Optional[np.ndarray] = None
    subscribers: List[Subscription] = field(default_factory=list)
    yolo_rgb: Optional[np.ndarray] = None
    downsample_factor: int = 4


class CameraViewerNode(Node):
    def __init__(self, camera_id):
        super().__init__(f"camera_viewer_{camera_id}")
        self.bridge = CvBridge()

        # Initialize image holders for both cameras
        self.rgb = None
        self.depth = None

        self.camera = Camera(name=camera_id)

        # Subscribe to topics
        rgb_topic = f"/camera/{self.camera.name}/rgb"
        depth_topic = f"/camera/{self.camera.name}/depth"
        yolo_topic = f"/person/{self.camera.name}/box_coords"

        self.camera.subscribers.append(
            self.create_subscription(
                Image, rgb_topic, self.image_callback(self.camera, "rgb"), 10
            )
        )

        self.camera.subscribers.append(
            self.create_subscription(
                Image, depth_topic, self.image_callback(self.camera, "depth"), 10
            )
        )

        self.camera.subscribers.append(
            self.create_subscription(
                Detection2DArray, yolo_topic, self.yolo_callback(self.camera), 10
            )
        )

        self.get_logger().info(f"Subscribed to {rgb_topic} and {depth_topic} and {yolo_topic}")

        # Window name
        self.window_name = f"Camera {self.camera.name} View"

        # Create a timer for displaying the view
        FPS = 10
        self.display_timer = self.create_timer(1.0 / FPS, self.display_view)  # ~30fps

    def image_callback(self, camera: Camera, image_type: str):
        """Callback for compressed image frames."""

        def callback(msg):
            try:
                if image_type == "rgb":
                    img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                else:
                    img = self.bridge.imgmsg_to_cv2(msg, "passthrough")

                DOWNSAMPLE_FACTOR = camera.downsample_factor
                img = cv2.resize(
                    img,
                    dsize=(
                        img.shape[1] // DOWNSAMPLE_FACTOR,
                        img.shape[0] // DOWNSAMPLE_FACTOR,
                    ),
                )

                if image_type == "rgb":
                    camera.rgb = img
                else:
                    camera.depth = img
            except Exception as e:
                self.get_logger().error(
                    f"Failed to process {image_type} image from {camera.name}: {e}"
                )
            self.get_logger().debug(f"Processed {image_type} image from {camera.name}")

        return callback

    def yolo_callback(self, camera: Camera):
        """Callback for YOLO Detection2DArray messages."""
        YOLO_TIMEOUT = 1.0  # seconds
        DOWNSAMPLE_FACTOR = camera.downsample_factor

        def clear_detections():                         
            if camera.rgb is not None:
                camera.yolo_rgb = camera.rgb.copy()

        def callback(msg):
            try:
                if camera.rgb is None:
                    self.get_logger().warn(f"No RGB image available for {camera.name}, skipping YOLO overlay")
                    return

                # Reset the stale timer on every new message   # <-- ADD THIS
                if hasattr(camera, '_yolo_timer') and camera._yolo_timer is not None:
                    camera._yolo_timer.cancel()
                camera._yolo_timer = self.create_timer(YOLO_TIMEOUT, clear_detections)

                img = camera.rgb.copy()

                for detection in msg.detections:
                    cx = detection.bbox.center.position.x / camera.downsample_factor
                    cy = detection.bbox.center.position.y / camera.downsample_factor
                    w = detection.bbox.size_x / camera.downsample_factor
                    h = detection.bbox.size_y / camera.downsample_factor

                    x1 = int(cx - w / 2)
                    y1 = int(cy - h / 2)
                    x2 = int(cx + w / 2)
                    y2 = int(cy + h / 2)

                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    if detection.results:
                        best = max(detection.results, key=lambda r: r.hypothesis.score)
                        label = f"{best.hypothesis.class_id}: {best.hypothesis.score:.2f}"
                        cv2.putText(
                            img, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
                        )

                camera.yolo_rgb = img

            except Exception as e:
                self.get_logger().error(
                    f"Failed to process YOLO detections for {camera.name}: {e}"
                )
            self.get_logger().debug(f"Processed YOLO detections for {camera.name}")

        return callback

    def process_camera_view(self, rgb_img, depth_img, camera_name, yolo_rgb=None):
        """Process a single camera's view (RGB + Depth side by side)."""
        display_rgb = rgb_img
        if(yolo_rgb is not None):
            display_rgb = yolo_rgb

        if display_rgb is not None and depth_img is not None:
            # Convert depth to color map
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET
            )

            # Resize depth to match RGB
            if display_rgb.shape[:2] != depth_colormap.shape[:2]:
                depth_colormap = cv2.resize(
                    depth_colormap, (display_rgb.shape[1], display_rgb.shape[0])
                )

            # Combine RGB and depth side by side
            combined = cv2.hconcat([display_rgb, depth_colormap])

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
                combined,
                "YOLO" if yolo_rgb is not None else "RGB",
                (30, 80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                combined,
                "Depth",
                (display_rgb.shape[1] + 30, 80),
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
        view = self.process_camera_view(
            self.camera.rgb, self.camera.depth, self.camera.name, self.camera.yolo_rgb
        )
        if view is not None:
            display_img = view

        if display_img is not None:
            cv2.imshow(self.window_name, display_img)
            cv2.waitKey(1)


def load_cameras_yaml(path):
        if not os.path.exists(path):
            raise FileNotFoundError("Noooooo! No yaml path exists :(")

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        return data

def main(args=None):
    rclpy.init(args=args)

    argv = remove_ros_args(args if args is not None else None)
    camera_name = argv[1] if len(argv) > 1 else None
    if not camera_name:
        print("Usage: ros2 run perception camera_viewer <camera_name>")
        rclpy.shutdown()
        return

    node = CameraViewerNode(camera_name)
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
