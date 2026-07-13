from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="thrusters",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="simulation",
                executable="sensors",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="simulation",
                executable="path_bridge",
                arguments=["--ros-args", "--log-level", "info"],
            ),
        ]
    )
