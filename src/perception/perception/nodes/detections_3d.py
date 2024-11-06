#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import depthai as dai
import time
import cv2
import numpy as np

MAX_DISTANCE = 3.0  # Maximum depth distance in meters


class To3DFrom2D(Node):
    def __init__(self):
        super().__init__("to3dfrom2d")
        self.get_logger().info("start of node")
        self.bridge = CvBridge()

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, "oak/depth/image_raw", self.img_sub_callback, 10
        )
        self.det_sub = self.create_subscription(
            Float32MultiArray,  # Assuming detection bounding boxes are published as a list of floats
            "detections2d",
            self.det_sub_callback,
            10,
        )

        # Publisher for 3D bounding box data
        self.bb_3d_pub = self.create_publisher(Float32MultiArray, "bounding_box_3d", 10)

        self.get_logger().info("subscribers and publisher created")

        # Variables to store the latest depth map and detections
        self.depth_map = None
        self.detections = []

    def img_sub_callback(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().info("Depth map received")
        if self.detections:  # Ensure detections are not empty
            self.run_filter()

    def det_sub_callback(self, msg):
        self.detections = np.array(msg.data).reshape(-1, 4)
        self.get_logger().info("Detections received")
        if self.depth_map is not None:  # Ensure depth_map is set
            self.run_filter()

    def run_filter(self):
        if self.depth_map is None or not self.detections:
            self.get_logger().info("Skipping run_filter due to missing data.")
            return
        self.get_logger().info("gets into run_filter")
        # For each detection bounding box, filter and compute 3D points
        for box in self.detections:
            x_center, y_center, box_width, box_height = box
            top_left_x = int(x_center - box_width / 2)
            top_left_y = int(y_center - box_height / 2)
            bottom_right_x = int(x_center + box_width / 2)
            bottom_right_y = int(y_center + box_height / 2)

            # Ensure the depth map is available
            if self.depth_map is None:
                self.get_logger().error("Depth map not available.")
                continue

            # Crop the region of interest in the depth map
            cropped_depth = self.depth_map[
                top_left_y:bottom_right_y, top_left_x:bottom_right_x
            ]
            points_3d = []

            self.get_logger().info("crop successful")

            for i in range(cropped_depth.shape[0]):
                for j in range(cropped_depth.shape[1]):
                    depth_value = cropped_depth[i, j]

                    # Filter out points based on max distance threshold
                    if depth_value > MAX_DISTANCE or depth_value <= 0:
                        continue

                    # Compute 3D coordinates from pixel coordinates and depth
                    x_3d, y_3d, z_3d = self.pixel_to_3d(
                        top_left_x + j, top_left_y + i, depth_value
                    )
                    points_3d.append([x_3d, y_3d, z_3d])

            # Find and publish optimal bounding box from 3D points
            self.compute_3d_bounding_box(points_3d)

    def pixel_to_3d(self, x, y, depth):
        # Intrinsic camera parameters (replace these with your camera's actual parameters)
        fx, fy = 600, 600  # Focal lengths in pixels
        cx, cy = 320, 240  # Principal point (image center)

        # Convert pixel coordinates (x, y) to normalized device coordinates
        x_3d = (x - cx) * depth / fx
        y_3d = (y - cy) * depth / fy
        z_3d = depth

        return x_3d, y_3d, z_3d

    def compute_3d_bounding_box(self, points_3d):
        if not points_3d:
            self.get_logger().warning(
                "No valid 3D points found for bounding box computation."
            )
            return

        # Convert list of 3D points to numpy array
        points_3d = np.array(points_3d)

        # Compute the min and max points for the bounding box in each dimension
        min_x, min_y, min_z = np.min(points_3d, axis=0)
        max_x, max_y, max_z = np.max(points_3d, axis=0)

        # Define the bounding box center and size
        center_x, center_y, center_z = (
            (min_x + max_x) / 2,
            (min_y + max_y) / 2,
            (min_z + max_z) / 2,
        )
        size_x, size_y, size_z = max_x - min_x, max_y - min_y, max_z - min_z

        # Publish bounding box data as a ROS message
        bb_msg = Float32MultiArray()
        bb_msg.data = [center_x, center_y, center_z, size_x, size_y, size_z]
        self.bb_3d_pub.publish(bb_msg)

        # Log the bounding box details
        self.get_logger().info(
            f"Published 3D bounding box: center=({center_x}, {center_y}, {center_z}), "
            f"size=({size_x}, {size_y}, {size_z})"
        )

    def run(self):
        # Call run_filter in a loop to process new detections and depth maps
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)

    to_3d_from_2d_node = To3DFrom2D()
    to_3d_from_2d_node.run()

    to_3d_from_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
