from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrust_generator",
                parameters=["params/thrusters.yaml"],
            )
        ]
    )
