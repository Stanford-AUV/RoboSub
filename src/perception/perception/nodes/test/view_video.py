import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import cv2
import numpy as np


SMALL_FACTOR = 0.1


class ViewVideo(Node):
    detections: list[Detection2D]

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

    def rgb_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Overlay bounding boxes
            self.overlay_bbox(frame)

            # Display the frame
            cv2.imshow("RGB Video Stream", frame)

            # Wait for a short time and check for the 'q' key to close the window
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error in RGB callback: {e}")

    def depth_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (depth)
            depth_frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

            # Normalize depth for better visualization
            ratio = 255 / depth_frame.max()  # Dynamically calculate max depth
            depth_frame_tmp = (depth_frame * ratio).astype(np.uint8)
            depth_frame_color_map = cv2.applyColorMap(depth_frame_tmp, cv2.COLORMAP_JET)

            # Overlay bounding boxes
            self.overlay_bbox(depth_frame_color_map)

            # Display the depth frame
            cv2.imshow("Depth Video Stream", depth_frame_color_map)

            # Wait for a short time and check for the 'q' key to close the window
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error in Depth callback: {e}")

    def detections_2d_callback(self, msg: Detection2DArray):
        self.detections = msg.detections

    def overlay_bbox(self, frame: np.ndarray):
        # Draw bounding boxes and labels on the frame
        if not self.detections:
            return

        for detection in self.detections:
            try:
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
            except Exception as e:
                self.get_logger().error(f"Error overlaying bbox: {e}")

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        rclpy.shutdown()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ViewVideo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()
