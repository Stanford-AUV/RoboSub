from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="thrusters",
                parameters=[global_params],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="simulation",
                executable="sensors",
                parameters=[global_params],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="simulation",
                executable="path_bridge",
                parameters=[global_params],
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
