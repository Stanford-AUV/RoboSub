from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="detection_node_3d",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="detection_transformer",
                parameters=[],
            ),
        ]
    )
