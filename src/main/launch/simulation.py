from launch import LaunchDescription
from launch_ros.actions import Node
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="thrusters",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            DeclareLaunchArgument("use_ground_truth", default_value="false"),
            Node(
                package="simulation",
                executable="ground_truth",
                arguments=["--ros-args", "--log-level", "warn"],
                parameters=[{"use_ground_truth": ParameterValue(LaunchConfiguration("use_ground_truth"), value_type=bool)}],
            ),
            Node(
                package="simulation",
                executable="path_bridge",
                arguments=["--ros-args", "--log-level", "info"],
            ),
            Node(
                package="simulation",
                executable="imu",
            ),
            Node(
                package="simulation",
                executable="arduino",
            ),
            Node(
                package="simulation",
                executable="oak",
            ),
        ]
    )
