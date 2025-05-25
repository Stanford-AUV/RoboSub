#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        
        # Initialize image holder
        self.compressed_image = None
        
        # Subscribe to COMPRESSED topic
        self.subscription_compressed = self.create_subscription(
            CompressedImage,
            '/oak/rgb/image_rect/compressed',
            self.compressed_image_callback,
            10
        )
        
        # Create a timer for displaying the view
        self.display_timer = self.create_timer(0.03, self.display_view)  # ~30fps

    def compressed_image_callback(self, msg):
        """Callback for compressed image frames."""
        try:
            self.compressed_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed image: {e}")
            
    def display_view(self):
        """Display compressed image."""
        if self.compressed_image is None:
            return
            
        # Display the image
        cv2.imshow("Camera Feed (COMPRESSED)", self.compressed_image)
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

if __name__ == '__main__':
    main()
