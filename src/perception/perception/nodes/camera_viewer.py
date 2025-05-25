#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewerNode(Node):
    def __init__(self):
        super().__init__('camera_viewer_node')
        self.bridge = CvBridge()
        
        # Initialize image holders
        self.raw_image = None
        self.compressed_image = None
        
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
        
        # Create a timer for displaying the combined view
        self.display_timer = self.create_timer(0.03, self.display_combined_view)  # ~30fps

    def image_callback(self, msg):
        """Callback for raw image frames."""
        try:
            self.raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

    def compressed_image_callback(self, msg):
        """Callback for compressed image frames."""
        try:
            self.compressed_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert compressed image: {e}")
            
    def display_combined_view(self):
        """Display both raw and compressed images side by side."""
        if self.raw_image is None or self.compressed_image is None:
            return
            
        # Ensure both images have the same width for vertical stacking
        h1, w1 = self.raw_image.shape[:2]
        h2, w2 = self.compressed_image.shape[:2]
        
        # Resize to match widths if necessary
        if w1 != w2:
            # Resize the wider image to match the narrower one
            if w1 > w2:
                aspect_ratio = h1 / w1
                new_width = w2
                new_height = int(aspect_ratio * new_width)
                self.raw_image = cv2.resize(self.raw_image, (new_width, new_height))
            else:
                aspect_ratio = h2 / w2
                new_width = w1
                new_height = int(aspect_ratio * new_width)
                self.compressed_image = cv2.resize(self.compressed_image, (new_width, new_height))
        
        # Add labels to the images
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.raw_image, 'RAW', (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(self.compressed_image, 'COMPRESSED', (10, 30), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
        
        # Combine images vertically (stacked)
        combined_img = np.vstack((self.raw_image, self.compressed_image))

        # Display the combined image
        cv2.imshow("Combined Camera Feeds (RAW on top, COMPRESSED on bottom)", combined_img)
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
