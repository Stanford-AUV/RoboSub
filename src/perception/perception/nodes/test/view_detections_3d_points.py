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

        # Initialization (outside the update function)
        self.scatter = None  # Define this in the class init or setup
        self.legend_items = {}  # Store legend handles

        # In the function where detections are processed
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
            points = np.array([[p.x, p.y, p.z] for p in detection.points])
            downsampled_points = points[:: len(points) // 100]  # Take every 100th point

            # Aggregate points and labels for bulk updating
            all_points.append(downsampled_points)
            all_labels.extend([label_text] * len(downsampled_points))

        # Combine all points into one array for faster plotting
        if all_points:
            combined_points = np.vstack(all_points)  # Shape (N, 3)
        else:
            combined_points = np.empty((0, 3))

        # Plot or update points
        if self.scatter is None:
            # Initialize scatter plot
            self.scatter = self.ax.scatter(
                combined_points[:, 0],
                combined_points[:, 1],
                combined_points[:, 2],
                c="blue",  # Color can vary
                marker="o",
            )
        else:
            # Update scatter plot data
            self.scatter._offsets3d = (
                combined_points[:, 0],
                combined_points[:, 1],
                combined_points[:, 2],
            )

        # Update legend (if required)
        self.ax.legend(
            self.legend_items.values(), self.legend_items.keys(), loc="upper right"
        )
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        plt.pause(0.001)  # Pause briefly for real-time updates


def main(args=None):
    rclpy.init(args=args)
    node = ViewDetections3DPointsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
