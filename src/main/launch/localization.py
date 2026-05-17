import os
from launch import LaunchDescription
from launch_ros.actions import Node

ekf_params = "/workspaces/Robosub/src/localization/localization/nodes/ekf.yaml"

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="hardware",
            executable="sensors",
            name="sensors",
            parameters=[global_params],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[ekf_params],  # <-- changed
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_imu_tf",
            arguments=[
                "-0.1525", "-0.02", "0.1375",
                "-1.57079632679", "0.0", "1.57079632679",
                "base_link", "imu_frame",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_dvl_tf",
            arguments=[
                "-0.105", "0.0", "-0.0625",
                "0.0", "0.0", "0.0",
                "base_link", "dvl_frame",
            ],
        ),
    ])