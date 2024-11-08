#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from vision_msgs.msg import Detection2DArray

MAX_DISTANCE = 10000  # Maximum depth distance in mm


class To3DFrom2D(Node):
    def __init__(self):
        super().__init__("to3dfrom2d")
        self.bridge = CvBridge()

        # Set depth image resolution (replace with actual resolution)
        self.depth_width = 1280  # Example depth image width
        self.depth_height = 800  # Example depth image height

        # Set the RGB image resolution (based on detection input)
        self.rgb_width = 4032  # Example RGB image width
        self.rgb_height = 3040  # Example RGB image height

        # Compute scaling factors
        self.scale_x = self.depth_width / self.rgb_width
        self.scale_y = self.depth_height / self.rgb_height

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

    def img_sub_callback(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.detections:  # Ensure detections are not empty
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

        if self.depth_map is not None:
            self.run_filter()

    def run_filter(self):
        if self.depth_map is None or not self.detections:
            return

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

            self.compute_3d_bounding_box(points_3d)

    def pixel_to_3d(self, x, y, depth):
        fx, fy = 600, 600  # Focal lengths in pixels (example)
        cx, cy = 320, 240  # Principal point

        x_3d = (x - cx) * depth / fx
        y_3d = (y - cy) * depth / fy
        z_3d = depth

        return x_3d, y_3d, z_3d

    def compute_3d_bounding_box(self, points_3d):
        points_3d = np.array(points_3d)

        if points_3d.size == 0:
            self.get_logger().error("Empty array")
            return

        min_x, min_y, min_z = np.min(points_3d, axis=0)
        max_x, max_y, max_z = np.max(points_3d, axis=0)

        center_x, center_y, center_z = (
            (min_x + max_x) / 2,
            (min_y + max_y) / 2,
            (min_z + max_z) / 2,
        )
        size_x, size_y, size_z = max_x - min_x, max_y - min_y, max_z - min_z

        bb_msg = Float32MultiArray()
        bb_msg.data = [center_x, center_y, center_z, size_x, size_y, size_z]
        self.get_logger().info(
            str([center_x, center_y, center_z, size_x, size_y, size_z])
        )
        self.bb_3d_pub.publish(bb_msg)

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
