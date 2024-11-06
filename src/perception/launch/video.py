from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="test_video",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="test_view_video",
                output="screen",
                parameters=[],
            ),
        ]
    )
