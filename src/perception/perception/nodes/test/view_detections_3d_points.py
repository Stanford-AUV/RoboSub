import rclpy
from rclpy.node import Node
import numpy as np
from typing import List
from msgs.msg import Detection3DPointsArray, Detection3DPoints
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from threading import Thread
import time


class ViewDetections3DPointsNode(Node):
    def __init__(self, app):
        super().__init__("view_detections_3d_points")
        self.get_logger().info("ViewDetections3DPointsNode with Dash has been created!")

        self.sub = self.create_subscription(
            Detection3DPointsArray, "detections3d_points", self.callback, 10
        )
        self.app = app
        # Initialize shared data (default empty values)
        self.detections_data = {"points": np.empty((0, 3)), "colors": [], "labels": []}

    def callback(self, msg: Detection3DPointsArray):
        self.get_logger().info("Received detections")

        detections: List[Detection3DPoints] = msg.detections

        all_points = []
        all_labels = []
        all_colors = []

        for i, detection in enumerate(detections):
            # Extract label and confidence
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                label_text = f"{label}: {confidence:.2f}"
            else:
                label_text = "Unknown"
                label = -1  # Default for unknown classes

            # Extract points
            points = np.array([[p.x, p.y, p.z] for p in detection.points])

            # Skip if no points are available
            if points.shape[0] == 0:
                continue

            # Downsample points dynamically
            downsample_factor = max(1, len(points) // 100)  # Avoid zero division
            downsampled_points = points[::downsample_factor]

            # Aggregate points and assign a color per label
            all_points.append(downsampled_points)
            color = f"rgb(255, 0, 0)"  # TODO: Assign color based on label
            all_labels.extend([label_text] * len(downsampled_points))
            all_colors.extend([color] * len(downsampled_points))

        # Combine all points for efficient plotting
        if all_points:
            combined_points = np.vstack(all_points)  # Shape (N, 3)
        else:
            combined_points = np.empty((0, 3))
            all_colors = []
            all_labels = []

        # Update shared data
        self.detections_data = {
            "points": combined_points,
            "colors": all_colors,
            "labels": all_labels,
        }


def create_dash_app(node):
    app = Dash(__name__)

    app.layout = html.Div(
        [
            html.H1("3D Point Cloud Viewer", style={"textAlign": "center"}),
            dcc.Graph(id="3d-plot"),
            dcc.Interval(
                id="update-interval", interval=500, n_intervals=0
            ),  # Update every 500ms
        ]
    )

    @app.callback(
        Output("3d-plot", "figure"),
        [Input("update-interval", "n_intervals")],
    )
    def update_plot(n):
        data = node.detections_data
        points = data["points"]
        colors = data["colors"]
        labels = data["labels"]

        if points.shape[0] > 0:
            trace = go.Scatter3d(
                x=points[:, 0],
                y=points[:, 1],
                z=points[:, 2],
                mode="markers",
                marker=dict(
                    size=5,
                    color=colors,  # Assign colors to each point
                    opacity=0.8,
                ),
                text=labels,  # Show labels on hover
            )
            layout = go.Layout(
                scene=dict(
                    xaxis_title="X (Horizontal)",
                    yaxis_title="Y (Vertical)",
                    zaxis_title="Z (Depth)",
                    # xaxis=dict(range=[-10, 10]),  # Adjust based on your environment
                    # yaxis=dict(range=[-10, 10]),
                    # zaxis=dict(range=[0, 20]),
                    camera=dict(
                        up=dict(x=0, y=0, z=1),  # Z-axis as "up"
                        eye=dict(
                            x=0, y=0, z=-2
                        ),  # Position the camera along the Z-axis
                        center=dict(x=0, y=0, z=0),  # Center the camera at the origin
                    ),
                ),
                margin=dict(l=0, r=0, b=0, t=40),  # Compact layout
            )
            return go.Figure(data=[trace], layout=layout)
        else:
            return go.Figure()

    return app


def ros2_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = ViewDetections3DPointsNode(None)

    # Create Dash app and pass the ROS2 node
    app = create_dash_app(node)
    app.config["suppress_callback_exceptions"] = True

    # Set the node in the app
    app.server.node = node

    # Run ROS2 spin in a separate thread
    ros_thread = Thread(target=ros2_thread, args=(node,))
    ros_thread.start()

    # Run Dash app
    app.run_server(
        debug=True, use_reloader=False
    )  # Set `use_reloader=False` to prevent Dash reloading issues


if __name__ == "__main__":
    main()
