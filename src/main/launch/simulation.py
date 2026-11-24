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
            DeclareLaunchArgument("yaw_drift_deg_per_s", default_value="-0.07"),
            Node(
                package="simulation",
                executable="imu",
                parameters=[{"yaw_drift_deg_per_s": ParameterValue(LaunchConfiguration("yaw_drift_deg_per_s"), value_type=float)}],
            ),
            Node(
                package="simulation",
                executable="arduino",
            ),
            Node(
                package="simulation",
                executable="oak",
            ),
            Node(
                package="simulation",
                executable="gz_dvl_bridge",
            ),
            Node(
                package="simulation",
                executable="dvl",
            ),
            
        ]
    )
