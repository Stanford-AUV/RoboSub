from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument


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
                executable="path_loader",
                parameters=[global_params],
            ),
            Node(
                package="planning",
                executable="path_streamer",
                parameters=[global_params],
            ),
        ]
    )
