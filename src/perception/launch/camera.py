from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="objects_localizer",
                output="screen",
                parameters=[
                    {"view_detections": True},
                ],
            ),
            Node(
                package="perception",
                executable="test.view_detections_3d",
                output="screen",
                parameters=[],
            ),
        ]
    )
