from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription(
        [
             Node(
                package="hardware",
                executable="localization_test",
            ),
        ]
    )
