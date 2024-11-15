from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np


class ViewVideo(Node):
    detections: list[Detection2D]
    frame_dims: Optional[tuple]

    def __init__(self):
        super().__init__("view_video")
        self.get_logger().info("ViewVideo node has been created!")

        self.bridge = CvBridge()
        self.rgb_sub = self.create_subscription(
            Image, "oak/rgb/image_raw", self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, "oak/depth/image_raw", self.depth_callback, 10
        )
        self.detections_2d_sub = self.create_subscription(
            Detection2DArray, "detections2d", self.detections_2d_callback, 10
        )
        self.detections = []
        self.frame_dims = None

    def rgb_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.frame_dims = frame.shape

        # Overlay BBOX
        self.overlay_bbox(frame)

        # Display the frame
        cv2.imshow("Video Stream", frame)

        # Wait for a short time and check for the 'q' key to close the window
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quitting video display")
            rclpy.shutdown()

    def depth_callback(self, msg):
        if not self.frame_dims:
            return

        # Convert ROS Image message to OpenCV image (RGB representation of depth)
        depth_frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Normalization for better visualization
        ratio = (
            255 / 95
        )  # Can use cam_depth.initialConfig.getMaxDisparity() instead of 95
        depth_frame_tmp = (depth_frame * ratio).astype(np.uint8)
        depth_frame_color_map = cv2.applyColorMap(depth_frame_tmp, cv2.COLORMAP_JET)
        self.get_logger().info(f"Shape: {depth_frame_color_map.shape}")
        self.get_logger().info(f"Shape: {self.frame_dims}")
        depth_frame_color_map = cv2.resize(
            depth_frame_color_map, (self.frame_dims[1], self.frame_dims[0])
        )

        # Overlay BBOX
        self.overlay_bbox(depth_frame_color_map)

        # Display the depth frame
        cv2.imshow("Depth Video Stream", depth_frame_color_map)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quitting depth display")
            rclpy.shutdown()

    def detections_2d_callback(self, msg: Detection2DArray):
        self.detections = msg.detections

    def overlay_bbox(self, frame: np.ndarray):
        # Draw bounding boxes and labels on the frame
        for detection in self.detections:
            bbox = detection.bbox
            x_min = int(bbox.center.position.x - bbox.size_x / 2)
            y_min = int(bbox.center.position.y - bbox.size_y / 2)
            x_max = int(bbox.center.position.x + bbox.size_x / 2)
            y_max = int(bbox.center.position.y + bbox.size_y / 2)

            # Draw rectangle around detected object
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Display the label with confidence score
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                label_text = f"{label}: {confidence:.2f}"
                cv2.putText(
                    frame,
                    label_text,
                    (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )


def main(args=None):
    rclpy.init(args=args)
    node = ViewVideo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
