from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="imu",
                parameters=[global_params],
            ),
            # Node(
            #     package="hardware",
            #     executable="dvl",
            #     parameters=[global_params],
            # ),
            # Node(
            #     package="control",
            #     executable="thrust_generator",
            #     parameters=[global_params],
            #     arguments=["--ros-args", "--log-level", "warn"],
            # ),
            # Node(
            #     package="hardware",
            #     executable="thrusters",
            #     parameters=[global_params],
            #     arguments=["--ros-args", "--log-level", "warn"],
            # ),
            # Node(
            #     package="hardware",
            #     executable="arduino",
            #     parameters=[global_params],
            #     arguments=["--ros-args"],
            # ),
        ]
    )
