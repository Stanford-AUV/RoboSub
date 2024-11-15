#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from vision_msgs.msg import Detection2DArray
import depthai as dai
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from perception.utils.bounding_box import oriented_bounding_box

MAX_DISTANCE = 10000  # Maximum depth distance in mm


class To3DFrom2D(Node):
    def __init__(self):
        super().__init__("to3dfrom2d")
        self.bridge = CvBridge()

        # Set depth and RGB image resolution
        self.depth_width = 640
        self.depth_height = 400
        self.rgb_width = 4056
        self.rgb_height = 3040

        # Compute scaling factors
        self.scale_x = self.depth_width / self.rgb_width
        self.scale_y = self.depth_height / self.rgb_height

        # Retrieve camera intrinsics
        self.fx, self.fy, self.cx, self.cy = self.get_camera_intrinsics()

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, "oak/depth/image_raw", self.img_sub_callback, 10
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, "detections2d", self.det_sub_callback, 10
        )

        # Publisher for 3D bounding box data
        self.bb_3d_pub = self.create_publisher(Float32MultiArray, "bounding_box_3d", 10)

        # Variables to store the latest depth map and detections
        self.depth_map = None
        self.detections = []

    def get_camera_intrinsics(self):

        fx = 3221.85791015625
        # Default fx:
        fy = 3221.85791015625
        # Default fy:
        cx = 2105.731689453125
        # Default cx:
        cy = 1528.2215576171875
        # Default cy:

        # self.get_logger().info(
        #     f"Retrieved camera intrinsics - fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}"
        # )

        return fx, fy, cx, cy

    def img_sub_callback(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.run_filter()

    def det_sub_callback(self, msg):
        self.detections = []
        for detection in msg.detections:
            bounding_box = detection.bbox
            x_center = bounding_box.center.position.x * self.scale_x
            y_center = bounding_box.center.position.y * self.scale_y
            box_width = bounding_box.size_x * self.scale_x
            box_height = bounding_box.size_y * self.scale_y
            self.detections.append([x_center, y_center, box_width, box_height])

        self.run_filter()

    def run_filter(self):
        # self.get_logger().info("RUN FILTER")

        if self.depth_map is None:
            self.get_logger().info("Missing depth map")
            return

        if not self.detections:
            self.get_logger().info("Missing detections")
            return

        # self.get_logger().info("Run")

        for box in self.detections:
            x_center, y_center, box_width, box_height = box
            top_left_x = int(x_center - box_width / 2)
            top_left_y = int(y_center - box_height / 2)
            bottom_right_x = int(x_center + box_width / 2)
            bottom_right_y = int(y_center + box_height / 2)

            # Ensure bounding box is within the depth map
            cropped_depth = self.depth_map[
                max(top_left_y, 0) : min(bottom_right_y, self.depth_height),
                max(top_left_x, 0) : min(bottom_right_x, self.depth_width),
            ]
            if cropped_depth.size == 0:
                continue

            points_3d = []
            for i in range(cropped_depth.shape[0]):
                for j in range(cropped_depth.shape[1]):
                    depth_value = cropped_depth[i, j]
                    if depth_value > MAX_DISTANCE or depth_value <= 0:
                        continue

                    x_3d, y_3d, z_3d = self.pixel_to_3d(
                        top_left_x + j, top_left_y + i, depth_value
                    )
                    points_3d.append([x_3d, y_3d, z_3d])

            # self.plot_center_sphere()

            self.compute_3d_bounding_box(points_3d)

    def pixel_to_3d(self, x, y, depth):
        x_3d = (x - self.cx) * depth / self.fx
        y_3d = (y - self.cy) * depth / self.fy
        z_3d = depth
        return x_3d, y_3d, z_3d

    def compute_3d_bounding_box(self, points_3d):
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

        points_3d = np.array(points_3d)

        if len(points_3d) <= 1:
            self.get_logger().error("Empty array")
            return

        center, size, rotation_matrix = oriented_bounding_box(points_3d)

        self.get_logger().info(f"Center: {center} Size: {size}")
        # self.get_logger().info(f"Rotation Matrix: {rotation_matrix}")

        # TODO send msg

    def run(self):
        rclpy.spin(self)


def main(args=None):
    rclpy.init(args=args)
    to_3d_from_2d_node = To3DFrom2D()
    to_3d_from_2d_node.run()
    to_3d_from_2d_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
