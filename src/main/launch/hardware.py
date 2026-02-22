from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")
parameters_file_path = Path(
    get_package_share_directory("xsens_mti_ros2_driver"), "param", "xsens_mti_node.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("plot", default_value="false", description="Run sensors_plot for IMU/DVL visualization"),
            Node(
                package="xsens_mti_ros2_driver",
                executable="xsens_mti_node",
                name="xsens_mti_node",
                output="screen",
                parameters=[parameters_file_path],
                arguments=[],
            ),
            Node(
                package="hardware",
                executable="imu",
                parameters=[global_params],
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
                executable="sensors_plot",
                condition=IfCondition(LaunchConfiguration("plot")),
            ),
        ]
    )
