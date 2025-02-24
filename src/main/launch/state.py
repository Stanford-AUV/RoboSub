from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="sensors",
                name="sensors",
                parameters=[global_params],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[global_params],
            ),
        ]
    )
