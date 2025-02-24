#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()

        # Subscribe to RAW topic
        self.subscription_raw = self.create_subscription(
            Image,
            '/oak/rgb/image_rect',
            self.image_callback,
            10
        )

        # Subscribe to COMPRESSED topic
        self.subscription_compressed = self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_rect/compressed',
            self.compressed_image_callback,
            10
        )

    def image_callback(self, msg):
        """Callback for raw image frames."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Camera Feed (RAW)", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def compressed_image_callback(self, msg):
        """Callback for compressed image frames."""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            cv2.imshow("Camera Feed (COMPRESSED)", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed image: {e}")

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

if __name__ == '__main__':
    main()
