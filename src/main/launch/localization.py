import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Owned by the localization package, resolved through its share dir.
ekf_params = os.path.join(get_package_share_directory("localization"), "ekf.yaml")



def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="sensors",
                name="sensors",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params],  # <-- changed
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "-0.1525",
                    "-0.02",
                    "0.1375",
                    # Identity rotation since 2026-07-11: the mount rotation is
                    # burned into the MTi (RotSensor alignment), so IMU data
                    # axes are already vehicle-aligned. Translation is the
                    # physical lever arm and still applies.
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "imu_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_dvl_tf",
                arguments=[
                    "-0.105",
                    "0.0",
                    "-0.0625",
                    "0.0",
                    "0.0",
                    "0.0",
                    "base_link",
                    "dvl_frame",
                ],
            ),
        ]
    )
