from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

parameters_file_path = Path(
    get_package_share_directory("xsens_mti_ros2_driver"), "param", "xsens_mti_node.yaml"
)


def generate_launch_description():
    return LaunchDescription(
        [
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
            ),
            Node(
                package="hardware",
                executable="imu_plot",
            ),
        ]
    )
