from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="path_generator",
                parameters=[global_params],
            ),
            Node(
                package="planning",
                executable="waypoint_test",
                parameters=[global_params],
            )
        ]
    )
