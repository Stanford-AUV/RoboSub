import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Per-subsystem params (object_world_localizer): installed to the perception
# share dir and symlinked back to src by tools/symlink_yamls.sh at launch.
object_tracking_params = os.path.join(
    get_package_share_directory("perception"), "object_tracking.yaml"
)


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
            # Turns detections3d (camera frame) into a filtered odom-frame
            # goal on /object/<id>/world_position. Harmlessly idle until
            # object_localizer is re-enabled (needs ultralytics installed).
            Node(
                package="perception",
                executable="object_world_localizer",
                output="screen",
                parameters=[object_tracking_params],
            ),
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
