from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="camera_publisher",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="yolov8_ros_node",
                output="screen",
                parameters=[],
            ),
        ]
    )
