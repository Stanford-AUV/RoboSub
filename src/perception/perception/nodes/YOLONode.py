#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloV8ROSNode(Node):

    def __init__(self):
        super().__init__("yolov8_ros_node")

        # Load the YOLO model
        self.model = YOLO("yolo11n.pt")  # Ensure the correct model path

        # Initialize the CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

        # Create a subscription to the image topic (e.g., /camera/image_raw)
        self.subscription = self.create_subscription(
            Image, "oak/rgb/image_raw", self.image_callback, 10
        )

        # Log to indicate that the node has started
        self.get_logger().info(
            "YoloV8ROSNode is running and subscribing to image_raw topic"
        )

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Run YOLO model on the image
        results = self.model(cv_image)

        # Annotate the image with detections
        annotated_frame = results[0].plot()

        # Display the annotated frame (optional)
        cv2.imshow("YOLOv8 Detections", annotated_frame)
        cv2.waitKey(1)  # Wait for a brief moment to update the display

        # (Optional) Save the annotated image as an output file if required
        # cv2.imwrite('output_frame.jpg', annotated_frame)


def main(args=None):
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the YoloV8ROSNode
    node = YoloV8ROSNode()

    # Keep the node running, listening to messages
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
