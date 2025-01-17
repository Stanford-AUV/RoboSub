import rclpy
from rclpy.node import Node
from typing import List
import numpy as np
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3D, Detection3DArray
from dash import Dash, dcc, html
from dash.dependencies import Input, Output
import plotly.graph_objects as go
from threading import Thread

import json


def get_rectangle_surface(center: Pose, size_x, size_y):
    """
    Generate surface grid data for a rectangle in 3D space.
    The rectangle lies in the XY plane at z = center.z.
    """
    cx, cy, cz = center.position.x, center.position.y, center.position.z
    hx, hy = size_x / 2.0, size_y / 2.0  # Half-widths

    # Define the grid for the rectangle
    x = np.array([[cx - hx, cx + hx], [cx - hx, cx + hx]])
    y = np.array([[cy - hy, cy - hy], [cy + hy, cy + hy]])
    z = np.full_like(x, cz)  # z is constant

    return x, y, z


class ViewDetections3DNode(Node):
    def __init__(self, app):
        super().__init__("view_detections_3d")
        self.get_logger().info("ViewDetections3DNode with Dash has been created!")

        self.sub = self.create_subscription(
            Detection3DArray, "detections3d", self.callback, 10
        )
        self.app = app

        # Shared data: rectangle surfaces
        self.detections_data = {"rectangles": []}

    def callback(self, msg: Detection3DArray):
        detections: List[Detection3D] = msg.detections

        rectangles = []

        for detection in detections:
            bbox = detection.bbox
            center = bbox.center
            size_x = bbox.size.x
            size_y = bbox.size.y

            # Generate surface grid for this rectangle
            x, y, z = get_rectangle_surface(center, size_x, size_y)

            # Optional: Assign a color based on detection results
            color = "rgba(255, 0, 0, 0.8)"  # Default red with transparency
            if detection.results:
                label = detection.results[0].hypothesis.class_id
                if label == 1:  # Example: Use different colors for classes
                    color = "rgba(0, 255, 0, 0.8)"
                elif label == 2:
                    color = "rgba(0, 0, 255, 0.8)"
            rectangles.append({"x": x, "y": y, "z": z, "color": color})

        # Update shared data
        self.detections_data["rectangles"] = rectangles


def create_dash_app(node):
    app = Dash(__name__)

    app.layout = html.Div(
        [
            html.H1("Detections", style={"textAlign": "center"}),
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
        data = node.detections_data["rectangles"]

        fig_data = []

        # Add surfaces for each rectangle
        for rect in data:
            fig_data.append(
                go.Surface(
                    x=rect["x"],
                    y=rect["y"],
                    z=rect["z"],
                    colorscale=[[0, rect["color"]], [1, rect["color"]]],
                    showscale=False,  # Disable colorbar
                    opacity=0.8,  # Adjust transparency
                )
            )

        # Layout
        layout = go.Layout(
            scene=dict(
                xaxis=dict(
                    title="X",
                    range=[-2, 2],  # Fixed range for X-axis
                ),
                yaxis=dict(
                    title="Y",
                    range=[-2, 2],  # Fixed range for Y-axis
                ),
                zaxis=dict(
                    title="Z",
                    range=[0, 10],  # Fixed range for Z-axis
                ),
                camera=dict(
                    up=dict(x=0, y=1, z=0),  # Y-axis points up
                    eye=dict(x=0, y=0, z=-2),  # Look into the scene along +Z
                    center=dict(x=0, y=0, z=0),  # Center the camera at the origin
                ),
            ),
            margin=dict(l=0, r=0, b=0, t=40),  # Compact layout
        )

        return go.Figure(data=fig_data, layout=layout)

    return app


def ros2_thread(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    # Create ROS2 node
    node = ViewDetections3DNode(None)

    # Create Dash app and pass the ROS2 node
    app = create_dash_app(node)
    app.config["suppress_callback_exceptions"] = True

    # Set the node in the app
    app.server.node = node

    # Run ROS2 spin in a separate thread
    ros_thread = Thread(target=ros2_thread, args=(node,))
    ros_thread.start()

    # Run Dash app
    app.run_server(debug=True, use_reloader=False)


if __name__ == "__main__":
    main()
