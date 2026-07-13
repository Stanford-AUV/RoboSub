from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="sensors",
                name="sensors",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
            ),
        ]
    )
