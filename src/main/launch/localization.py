from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="sensors",
                name="sensors",
                parameters=[global_params],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_to_base_link",
                arguments=[
                    # x y z translation of your IMU from base_link
                    "-0.13005",  "-0.01366", "-0.03575",
                    # quaternion orientation of imu_link relative to base_link
                    "0", "0", "0", "1",
                    "imu_link", "base_link"
                ]
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[global_params],
            ),
        ]
    )
