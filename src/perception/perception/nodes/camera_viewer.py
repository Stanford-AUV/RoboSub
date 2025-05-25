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
        
        self.declare_parameter('camera_name', 'oak1')
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        
        self.subscription_raw_oak1 = self.create_subscription(
            Image,
            '/oak1/rgb/image_rect',
            self.image_callback_oak1,
            10
        )
        
        self.subscription_raw_oak2 = self.create_subscription(
            Image,
            '/oak2/rgb/image_rect',
            self.image_callback_oak2,
            10
        )
        
        self.subscription_depth_oak1 = self.create_subscription(
            Image,
            '/oak1/stereo/depth',
            self.depth_callback_oak1,
            10
        )
        
        self.subscription_depth_oak2 = self.create_subscription(
            Image,
            '/oak2/stereo/depth',
            self.depth_callback_oak2,
            10
        )
        
        self.get_logger().info(f"Camera viewer initialized, primary camera: {self.camera_name}")

    def image_callback_oak1(self, msg):
        """Callback for oak1 RGB image frames."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("OAK1 RGB", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert oak1 RGB image: {e}")

    def image_callback_oak2(self, msg):
        """Callback for oak2 RGB image frames."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("OAK2 RGB", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert oak2 RGB image: {e}")
            
    def depth_callback_oak1(self, msg):
        """Callback for oak1 depth image frames."""
        try:
            # Convert depth image (typically 16-bit single channel)
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            # Normalize for display
            min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
            if max_val > 0:
                cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                cv_image_display = cv_image_normalized.astype('uint8')
                # Apply colormap for better visualization
                cv_image_colormap = cv2.applyColorMap(cv_image_display, cv2.COLORMAP_JET)
                cv2.imshow("OAK1 Depth", cv_image_colormap)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert oak1 depth image: {e}")
    
    def depth_callback_oak2(self, msg):
        """Callback for oak2 depth image frames."""
        try:
            # Convert depth image (typically 16-bit single channel)
            cv_image = self.bridge.imgmsg_to_cv2(msg)
            # Normalize for display
            min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
            if max_val > 0:
                cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
                cv_image_display = cv_image_normalized.astype('uint8')
                # Apply colormap for better visualization
                cv_image_colormap = cv2.applyColorMap(cv_image_display, cv2.COLORMAP_JET)
                cv2.imshow("OAK2 Depth", cv_image_colormap)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to convert oak2 depth image: {e}")

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