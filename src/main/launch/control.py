from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrust_generator",
                parameters=[global_params],
            ),
            Node(
                package="control",
                executable="controller",
                parameters=[global_params],
            ),
            Node(
                package="control",
                executable="sim_tester",
                parameters=[global_params],
            ),
        ]
    )
