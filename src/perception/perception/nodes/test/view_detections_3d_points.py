import rclpy
from rclpy.node import Node
import numpy as np
from typing import List
from msgs.msg import Detection3DPointsArray, Detection3DPoints
import matplotlib.pyplot as plt


class ViewDetections3DPointsNode(Node):
    def __init__(self):
        super().__init__("view_detections_3d_points")
        self.get_logger().info("ViewVideo node has been created!")

        self.sub = self.create_subscription(
            Detection3DPointsArray, "detections3d_points", self.callback, 10
        )
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")

    def callback(self, msg: Detection3DPointsArray):
        self.ax.clear()

        self.get_logger().info("Received detections")

        detections: List[Detection3DPoints] = msg.detections

        all_points = []
        all_labels = []
        for detection in detections:
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                label_text = f"{label}: {confidence:.2f}"
            else:
                label_text = "Unknown"

            # Extract points and downsample
            points = np.array(
                [[p.x, p.y, p.z] for p in detection.points]
            )  # Reverse Y-axis for camera alignment
            downsampled_points = points[:: len(points) // 100]  # Downsample points

            # Aggregate points and labels for bulk updating
            all_points.append(downsampled_points)
            all_labels.extend([label_text] * len(downsampled_points))

        # Combine all points into one array for faster plotting
        if all_points:
            combined_points = np.vstack(all_points)  # Shape (N, 3)
        else:
            combined_points = np.empty((0, 3))

        # Plot points with camera-aligned axes
        self.ax.scatter(
            combined_points[:, 0],  # X-axis: Horizontal
            combined_points[:, 1],  # Y-axis: Vertical
            combined_points[:, 2],  # Z-axis: Depth
            c="blue",  # You can vary the color per label
            marker="o",
        )

        # Update plot labels and set view to camera perspective
        self.ax.set_xlabel("X (Horizontal)")
        self.ax.set_ylabel("Y (Vertical)")
        self.ax.set_zlabel("Z (Depth)")
        # self.ax.set_xlim([-10, 10])  # Adjust based on your expected range
        # self.ax.set_ylim([-10, 10])
        # self.ax.set_zlim([0, 20])

        # Set view to align with the camera
        self.ax.view_init(
            elev=0, azim=0, vertical_axis="y"
        )  # Camera view: Looking down Z-axis

        plt.pause(0.1)  # Pause briefly for real-time updates


def main(args=None):
    rclpy.init(args=args)
    node = ViewDetections3DPointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
