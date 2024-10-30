#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai as dai
import time
import cv2
import numpy as np

MAX_DISTANCE = 3

class To3DFrom2D(Node):
    def __init__(self):
        super().__init__("to3dfrom2d")
        self.img_sub = self.create_subscription(
            "img_sub",
            "oak/depth/image_raw",
            self.img_sub_callback,
            10
        )
        self.det_sub = self.create_subscription(
            "det_sub",
            "detections",
            self.det_sub_callback,
            10
        )
        self.bb_3d = None
        self.pub = self.create_publisher(self.bb_3d, "3d_bounding_box", 10)

    def img_sub_callback(self, msg):
        self.get_logger().info(msg.data)

    def det_sub_callback(self, msg):
        self.get_logger().info(msg.data)

    def get_depth_map(self):
        depth_map = None # TODO: subscribe to depth node and read in depth map


    def run_filter(self):
        box_list = self.det_sub.detections # should be 1 image, list of boxes
        for box in box_list:
            w, h = box.size
            x, y = box.center.position
            top_left = np.asarray([y - h/2, x - w/2])
            top_right = np.asarray([y - h/2, x + w/2])
            bottom_left = np.asarray([y + h/2, x - w/2])
            bottom_right = np.asarray([y + h/2, x + w/2])
            dm = np.asarray(self.get_depth_map())
            cropped = dm[y - h/2 : y + h/2, x - h/2 : x + h/2]
            for i in range(w):
                for j in range(h):
                    if cropped[i][j] > MAX_DISTANCE:
                        cropped[i][j] = -1
                    else:
                        # compute 3d points
            
            # bbox: center, size
            # center: Pose2D, theta
                # Pose2D: x, y
            # size: x, y
            # (0, 0) is top left


# DONE: Nodes subscribe to oak/depth/image_raw and detections
# For each detection:
# 1. Run a filter to isolate points that actually belong to the object in the 2d bounding box
# 2. Compute 3d points for all of these points
# 3. Plot 3d points in matplot lib (in camera points)
# 4. Find optimal bounding box
# 5. Plot bounding box

def main(args=None):
    rclpy.init(args=args)

    t3df2d = To3DFrom2D()
    t3df2d.run()

    t3df2d.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()