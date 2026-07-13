from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrust_generator",
            ),
            # Node(
            #     package="control",
            #     executable="controller",
            # ),
            Node(
                package="manual",
                executable="keyboard",
            ),
        ]
    )
