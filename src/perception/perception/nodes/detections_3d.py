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

            self.plot_center_sphere()

            self.compute_3d_bounding_box(points_3d)

    def pixel_to_3d(self, x, y, depth):
        x_3d = (x - self.cx) * depth / self.fx
        y_3d = (y - self.cy) * depth / self.fy
        z_3d = depth
        return x_3d, y_3d, z_3d

    def plot_center_sphere(self):
        # Calculate the center pixel coordinates in the depth image
        center_pixel_x = self.depth_width // 2
        center_pixel_y = self.depth_height // 2

        # Get the depth value at the center pixel and convert to meters if needed
        center_depth = self.depth_map[center_pixel_y, center_pixel_x]
        self.get_logger().info(f"Center depth: {center_depth}")

        # Check if depth is within a valid range
        if center_depth > 0 and center_depth <= MAX_DISTANCE:
            # Convert the center pixel’s 2D coordinates to 3D
            center_x_3d, center_y_3d, center_z_3d = self.pixel_to_3d(
                center_pixel_x, center_pixel_y, center_depth
            )

            # Log the 3D center coordinates
            self.get_logger().info(
                f"Center 3D coordinates: X: {center_x_3d}, Y: {center_y_3d}, Z: {center_z_3d}"
            )

            # Plotting the 3D view with the center sphere
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

            # Plot the sphere at the calculated 3D point for the center of the camera view
            self.plot_3d_sphere(ax, center_x_3d, center_y_3d, center_z_3d)

            # Set labels and limits for better visualization
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title("3D Center Point of Camera View")

            # Show the plot
            plt.show()
        else:
            # Log if the center depth is out of range
            self.get_logger().error(f"Center depth out of valid range: {center_depth}")

    def compute_3d_bounding_box(self, points_3d):
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

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

        # Publish the bounding box data
        bb_msg = Float32MultiArray()
        bb_msg.data = [center_x, center_y, center_z, size_x, size_y, size_z]
        self.get_logger().info(
            str([center_x, center_y, center_z, size_x, size_y, size_z])
        )
        self.bb_3d_pub.publish(bb_msg)

        # Plotting the 3D points and the bounding box
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plot the points in 3D
        ax.scatter(
            points_3d[:, 0],
            points_3d[:, 1],
            points_3d[:, 2],
            color="b",
            s=1,
            label="Points",
        )

        # Define the eight vertices of the bounding box
        vertices = np.array(
            [
                [min_x, min_y, min_z],
                [min_x, min_y, max_z],
                [min_x, max_y, min_z],
                [min_x, max_y, max_z],
                [max_x, min_y, min_z],
                [max_x, min_y, max_z],
                [max_x, max_y, min_z],
                [max_x, max_y, max_z],
            ]
        )

        # Define the edges of the bounding box
        edges = [
            [vertices[0], vertices[1], vertices[3], vertices[2]],  # Front face
            [vertices[4], vertices[5], vertices[7], vertices[6]],  # Back face
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Bottom face
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Top face
            [vertices[0], vertices[2], vertices[6], vertices[4]],  # Left face
            [vertices[1], vertices[3], vertices[7], vertices[5]],  # Right face
        ]

        # Add bounding box edges to the plot
        ax.add_collection3d(
            Poly3DCollection(
                edges, facecolors="orange", linewidths=1, edgecolors="r", alpha=0.1
            )
        )

        # Plot the center circle in the 2D image frame
        self.get_logger().info(f"before 2d")

        # self.plot_2d_center_circle()

        # Compute the center pixel of the depth image and plot it as a sphere in 3D
        center_pixel_x = self.depth_width // 2
        center_pixel_y = self.depth_height // 2
        center_depth = self.depth_map[center_pixel_y, center_pixel_x]
        self.get_logger().info(f"center_depth {center_depth}")

        # Convert the center pixel's 2D coordinates to 3D
        if center_depth > 0 and center_depth <= MAX_DISTANCE:
            self.get_logger().info(f"goes into if statement")
            center_x_3d, center_y_3d, center_z_3d = self.pixel_to_3d(
                center_pixel_x, center_pixel_y, center_depth
            )

            # Plot the sphere at the 3D center point location

        self.get_logger().info(
            f"center_x_3d{center_x_3d} center_y_3d {center_y_3d}, center_z_3d  {center_z_3d}"
        )
        self.plot_3d_sphere(ax, center_x_3d, center_y_3d, center_z_3d)
        self.get_logger().info(f)

        # Set labels and limits
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_title("3D Points with Bounding Box and Center Pixel Sphere")

        # Show the plot
        plt.legend()
        plt.show()

    def plot_2d_center_circle(self):
        # This function plots a red circle at the center of the 2D depth frame
        fig, ax = plt.subplots()
        ax.imshow(self.depth_map, cmap="gray")  # Display the depth image
        center_pixel_x = self.depth_width // 2
        center_pixel_y = self.depth_height // 2
        circle_radius = 50  # You can adjust this radius for visibility
        circle = plt.Circle(
            (center_pixel_x, center_pixel_y),
            circle_radius,
            color="red",
            fill=False,
            linewidth=2,
        )
        ax.add_patch(circle)
        plt.title("Depth Image with Center Circle")
        plt.show()

    def plot_3d_sphere(self, ax, x, y, z, radius=1):
        # This function plots a sphere at the given (x, y, z) coordinates in the 3D plot
        u, v = np.mgrid[0 : 2 * np.pi : 20j, 0 : np.pi : 10j]
        sphere_x = radius * np.cos(u) * np.sin(v) + x
        sphere_y = radius * np.sin(u) * np.sin(v) + y
        sphere_z = radius * np.cos(v) + z

        ax.plot_surface(
            sphere_x, sphere_y, sphere_z, color="red", alpha=0.6, rstride=1, cstride=1
        )

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
