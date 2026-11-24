import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
from cv_bridge import CvBridge

class SimOak(Node):
    """Stands in for perception's oak_node in sim: gz/<key>/rgb (rgb8) → /camera/<key>/rgb (bgr8), and gz/<key>/depth (32FC1, metres) → /camera/<key>/depth (16UC1, millimetres, 0 = no reading)."""
    def __init__(self):
        super().__init__("sim_oak")

        self.declare_parameter("camera_key", "oak_0")
        self.camera_key = self.get_parameter("camera_key").value
        self.bridge = CvBridge()

        self.rgb_publisher = self.create_publisher(Image, f"/camera/{self.camera_key}/rgb", 10)
        self.depth_publisher = self.create_publisher(Image, f"/camera/{self.camera_key}/depth", 10)

        self.create_subscription(Image, f"gz/{self.camera_key}/rgb", self.on_rgb, 10)
        self.create_subscription(Image, f"gz/{self.camera_key}/depth", self.on_depth, 10)

    def on_rgb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        out = self.bridge.cv2_to_imgmsg(img, "bgr8")
        out.header = msg.header
        self.rgb_publisher.publish(out)

    def on_depth(self, msg: Image):
        distances_m = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        distances_mm = np.where(np.isfinite(distances_m), distances_m * 1000.0, 0).clip(0, 65535).astype(np.uint16)
        out = self.bridge.cv2_to_imgmsg(distances_mm, "16UC1")
        out.header = msg.header
        self.depth_publisher.publish(out)
    

def main(args=None):
    rclpy.init(args=args)
    sim_oak_node = SimOak()
    rclpy.spin(sim_oak_node)
    sim_oak_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()