import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Resolves now that localization is a real colcon package with a share dir.
EKF_PARAMS = os.path.join(get_package_share_directory("localization"), "ekf.yaml")


def generate_launch_description():
    """EKF only - no sensor drivers, no static TFs.

    For the full state-estimation bring-up (sensors + EKF + base_link TFs) use
    `ros2 launch main localization.py`.
    """
    return LaunchDescription(
        [
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[EKF_PARAMS],
            ),
        ]
    )
