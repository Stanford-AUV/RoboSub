from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

# NOTE: the Xsens driver (xsens_mti_node) + the hardware `imu` node were moved
# out to imu.py so launch_sub.sh can start them FIRST and let the Xsens onboard
# filter settle before the rest of the stack comes up. Launch imu.py (or use
# main.py, which includes it) alongside this file for a full IMU-in bringup.


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "plot",
                default_value="true",
                description="Run sensors_plot for IMU/DVL visualization "
                "(on by default; disable with plot:=false)",
            ),
            Node(
                package="hardware",
                executable="dvl",
            ),
            Node(
                package="control",
                executable="thrust_generator",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="hardware",
                executable="thrusters",
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="hardware",
                executable="arduino",
                arguments=["--ros-args"],
            ),
            Node(
                package="hardware",
                executable="depth",
            ),
            Node(
                package="hardware",
                executable="sensors_plot",
                condition=IfCondition(LaunchConfiguration("plot")),
            ),
        ]
    )
