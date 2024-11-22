import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ViewVideo(Node):
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

    def rgb_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display the frame
        cv2.imshow("Video Stream", frame)

        # Wait for a short time and check for the 'q' key to close the window
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quitting video display")
            rclpy.shutdown()

    def depth_callback(self, msg):
        # Convert ROS Image message to OpenCV image (RGB representation of depth)
        depth_frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")

        # Normalization for better visualization
        ratio = (
            255 / 95
        )  # Can use cam_depth.initialConfig.getMaxDisparity() instead of 95
        depth_frame_tmp = (depth_frame * ratio).astype(np.uint8)
        depth_frame_color_map = cv2.applyColorMap(depth_frame_tmp, cv2.COLORMAP_JET)

        # Display the depth frame
        cv2.imshow("Depth Video Stream", depth_frame_color_map)

        # Quit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Quitting depth display")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ViewVideo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
