from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(
    os.path.dirname(__file__), "..", "..", "main", "launch", "params", "global.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="pid_control",
                parameters=[
                    global_params
                ],
            ),
        ]
    )
