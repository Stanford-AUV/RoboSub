import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Resolve ekf.yaml relative to this launch file's real location (works on host +
# in --symlink-install; the old hardcoded /workspaces/RoboSub path was Docker-only).
ekf_params = os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    "..", "..", "localization", "localization", "nodes", "ekf.yaml",
)



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
