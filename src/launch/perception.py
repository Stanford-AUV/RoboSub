from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",  # Replace with your package name
                executable="image_publisher",  # The name of the executable (should match the entry point defined in setup.py)
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",  # Replace with your package name
                executable="yolov8_ros_node",  # The name of the executable (should match the entry point defined in setup.py)
                output="screen",
                parameters=[],
            ),
        ]
    )
