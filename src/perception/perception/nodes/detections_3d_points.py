import rclpy
from typing import List
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
)
from geometry_msgs.msg import Point
from msgs.msg import Detection3DPointsArray, Detection3DPoints

from perception.utils.bounding_box import oriented_bounding_box


class Detections3DPointsNode(Node):
    def __init__(self):
        super().__init__("detections_3d_points")
        self.bridge = CvBridge()

        # Retrieve camera intrinsics
        self.fx_px, self.cx, self.cy = self.get_camera_intrinsics()

        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, "oak/depth/image_raw", self.img_sub_callback, 10
        )
        self.det_sub = self.create_subscription(
            Detection2DArray, "detections2d", self.det_sub_callback, 10
        )

        # Publisher for 3D points
        self.det_pub = self.create_publisher(
            Detection3DPointsArray, "detections3d_points", 10
        )

        # Variables to store the latest depth map and detections
        self.depth_map = None
        self.detections: List[Detection2D] = []

    def get_camera_intrinsics(self):

        fx_px = 3050.27978515625
        # Default f:
        cx = 1993.59228515625
        # Default cx:
        cy = 1087.78369140625
        # Default cy:

        # self.get_logger().info(
        #     f"Retrieved camera intrinsics - fx: {fx}, fy: {fy}, cx: {cx}, cy: {cy}"
        # )

        return fx_px, cx, cy

    def img_sub_callback(self, msg):
        self.depth_map = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.run_filter()

    def det_sub_callback(self, msg: Detection2DArray):
        self.detections = msg.detections

        self.run_filter()

    def run_filter(self):
        if self.depth_map is None:
            self.get_logger().info("Missing depth map")
            return

        if not self.detections:
            self.get_logger().info("Missing detections")
            return

        detections3d_points_array = Detection3DPointsArray()
        detections3d_points_array.header.stamp = self.get_clock().now().to_msg()

        for detection in self.detections:
            bounding_box = detection.bbox
            x_center = bounding_box.center.position.x
            y_center = bounding_box.center.position.y
            box_width = bounding_box.size_x
            box_height = bounding_box.size_y

            height, width = self.depth_map.shape
            left_x = max(0, int(x_center - box_width / 2))
            top_y = max(0, int(y_center - box_height / 2))
            right_x = min(width, int(x_center + box_width / 2))
            bottom_y = min(height, int(y_center + box_height / 2))
            # points_3d = []
            # for y in range(top_y, bottom_y):
            #     for x in range(left_x, right_x):
            #         depth = self.depth_map[y, x]
            #         x_3d = (x - self.cx) * depth / self.fx
            #         y_3d = (y - self.cy) * depth / self.fy
            #         z_3d = depth
            #         points_3d.append([x_3d, y_3d, z_3d])
            # Vectorized version of the above:
            y_indices, x_indices = np.meshgrid(
                np.arange(top_y, bottom_y), np.arange(left_x, right_x), indexing="ij"
            )
            depths = self.depth_map[top_y:bottom_y, left_x:right_x] / 1000.0
            y_indices, x_indices = np.meshgrid(
                np.arange(top_y, bottom_y), np.arange(left_x, right_x), indexing="ij"
            )
            # depths = np.ones(depths.shape)
            valid_mask = depths > 0  # Filter invalid depths
            x_indices = x_indices[valid_mask]
            y_indices = y_indices[valid_mask]
            depths = depths[valid_mask]

            x_3d = (x_indices - self.cx) * depths / self.fx_px
            y_3d = (y_indices - self.cy) * depths / self.fx_px
            z_3d = depths
            points_3d = np.stack((x_3d, y_3d, z_3d), axis=-1)  # .reshape(-1, 3)

            detection3d = Detection3DPoints()
            detection3d.header = detection.header
            detection3d.id = detection.id
            detection3d.results = detection.results
            points = [Point(x=x, y=y, z=z) for x, y, z in points_3d]
            detection3d.points = points
            detections3d_points_array.detections.append(detection3d)

        self.det_pub.publish(detections3d_points_array)


def main(args=None):
    rclpy.init(args=args)
    node = Detections3DPointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
