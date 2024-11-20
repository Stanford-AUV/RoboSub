from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="camera",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="objects_detector",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="test.view_video",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="to3dfrom2d",
                output="screen",
                parameters=[],
            ),
        ]
    )
