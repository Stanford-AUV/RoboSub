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
global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


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
                parameters=[global_params],
            ),
            Node(
                package="control",
                executable="thrust_generator",
                parameters=[global_params],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="hardware",
                executable="thrusters",
                parameters=[global_params],
                arguments=["--ros-args", "--log-level", "warn"],
            ),
            Node(
                package="hardware",
                executable="arduino",
                parameters=[global_params],
                arguments=["--ros-args"],
            ),
            Node(
                package="hardware",
                executable="depth",
                parameters=[global_params],
            ),
            Node(
                package="hardware",
                executable="sensors_plot",
                condition=IfCondition(LaunchConfiguration("plot")),
            ),
        ]
    )
