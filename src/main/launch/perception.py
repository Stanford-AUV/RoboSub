from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # object_localizer disabled: ultralytics (YOLO) is not installed
            # in the robosub env, the node dies on import. Re-enable once
            # detection is back on the sub.
            # Node(
            #     package="perception",
            #     executable="object_localizer",
            #     output="screen",
            #     parameters=[],
            # ),
            Node(
                package="perception",
                executable="oak_node",
                output="screen",
                parameters=[],
            ),
            # Bottom-line heading anchor: publishes absolute yaw on
            # /heading_correction, fused by the EKF (pose1 in ekf.yaml).
            Node(
                package="perception",
                executable="heading_corrector",
                output="screen",
                parameters=[],
            ),
        ]
    )
