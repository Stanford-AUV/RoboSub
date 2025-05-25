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
        
        # Initialize image holders for both cameras
        self.oak1_rgb = None
        self.oak1_depth = None
        self.oak2_rgb = None
        self.oak2_depth = None
        
        # Subscribe to OAK1 topics
        self.oak1_rgb_sub = self.create_subscription(
            CompressedImage,
            '/oak1/rgb/image_rect/compressed',
            lambda msg: self.image_callback(msg, 'oak1', 'rgb'),
            10
        )
        
        self.oak1_depth_sub = self.create_subscription(
            CompressedImage,
            '/oak1/stereo/depth/compressed',
            lambda msg: self.image_callback(msg, 'oak1', 'depth'),
            10
        )
        
        # Subscribe to OAK2 topics
        self.oak2_rgb_sub = self.create_subscription(
            CompressedImage,
            '/oak2/rgb/image_rect/compressed',
            lambda msg: self.image_callback(msg, 'oak2', 'rgb'),
            10
        )
        
        self.oak2_depth_sub = self.create_subscription(
            CompressedImage,
            '/oak2/stereo/depth/compressed',
            lambda msg: self.image_callback(msg, 'oak2', 'depth'),
            10
        )
        
        # Window name
        self.window_name = "Dual Camera View"
        
        # Create a timer for displaying the view
        self.display_timer = self.create_timer(0.03, self.display_view)  # ~30fps

    def image_callback(self, msg, oak, image_type):
        """Callback for compressed image frames."""
        try:
            if image_type == 'rgb':
                img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
                if oak == 'oak1':
                    self.oak1_rgb = img
                else:  # oak2
                    self.oak2_rgb = img
            else:  # depth
                img = self.bridge.compressed_imgmsg_to_cv2(msg, 'mono16')
                if oak == 'oak1':
                    self.oak1_depth = img
                else:  # oak2
                    self.oak2_depth = img
        except Exception as e:
            self.get_logger().error(f"Failed to process {image_type} image from {oak}: {e}")
            
    def process_camera_view(self, rgb_img, depth_img, camera_name):
        """Process a single camera's view (RGB + Depth side by side)."""
        if rgb_img is not None and depth_img is not None:
            # Convert depth to color map
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_img, alpha=0.03),
                cv2.COLORMAP_JET
            )
            
            # Resize depth to match RGB
            if rgb_img.shape[:2] != depth_colormap.shape[:2]:
                depth_colormap = cv2.resize(depth_colormap, 
                                          (rgb_img.shape[1], rgb_img.shape[0]))
            
            # Combine RGB and depth side by side
            combined = cv2.hconcat([rgb_img, depth_colormap])
            
            # Add camera name label
            cv2.putText(combined, camera_name.upper(), (20, 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(combined, "RGB", (30, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(combined, "Depth", (rgb_img.shape[1] + 30, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            
            return combined
        return None
            
    def display_view(self):
        """Display both cameras' views stacked vertically."""
        # Process each camera's view
        oak1_view = self.process_camera_view(self.oak1_rgb, self.oak1_depth, 'OAK1')
        oak2_view = self.process_camera_view(self.oak2_rgb, self.oak2_depth, 'OAK2')
        
        # Stack the views vertically if both are available
        if oak1_view is not None and oak2_view is not None:
            # Make sure both views have the same width (resize oak2 to match oak1)
            if oak1_view.shape[1] != oak2_view.shape[1]:
                oak2_view = cv2.resize(oak2_view, (oak1_view.shape[1], oak2_view.shape[0]))
            
            # Stack vertically
            display_img = cv2.vconcat([oak1_view, oak2_view])
            
            # Add separator line
            sep_y = oak1_view.shape[0]
            cv2.line(display_img, (0, sep_y), (display_img.shape[1], sep_y), (0, 255, 0), 2)
            
            # Display the combined view
            cv2.imshow(self.window_name, display_img)
            cv2.waitKey(1)
        elif oak1_view is not None:
            cv2.imshow(self.window_name, oak1_view)
            cv2.waitKey(1)
        elif oak2_view is not None:
            cv2.imshow(self.window_name, oak2_view)
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
