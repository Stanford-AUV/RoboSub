from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

# IMU + Xsens driver ONLY. Split out of hardware.py so launch_sub.sh can bring
# the Xsens up first and let its onboard filter (AHS heading + orientation)
# converge for ~20s BEFORE the EKF / control start consuming it. hardware.py no
# longer starts these two nodes; main.py includes this file to keep the full
# bringup complete.
global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")
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
                parameters=[global_params],
            ),
        ]
    )
