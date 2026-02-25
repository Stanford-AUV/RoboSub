"""Launch aligned depth publisher + camera-agnostic object localizer (all nodes in one launch)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

global_params = os.path.join(
    os.path.dirname(__file__), "..", "..", "main", "launch", "params", "global.yaml"
)

VENV_PYTHON = "/home/ros/env/bin/python3 -u "


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_type",
                default_value="oak",
                description="Camera backend: oak or realsense",
            ),
            DeclareLaunchArgument(
                "camera_key",
                default_value="oak_0",
                description="Camera key from cameras config (e.g. oak_0, realsense_0)",
            ),
            DeclareLaunchArgument(
                "model_name",
                default_value="yolo-Weights/yolov8n.pt",
                description="YOLO model path for object_localizer",
            ),
            DeclareLaunchArgument(
                "object_id",
                default_value="person",
                description="Class name to localize (e.g. person)",
            ),
            DeclareLaunchArgument(
                "visualize_camera",
                default_value="false",
                description="Show cv2.imshow debug window for object_localizer",
            ),
            DeclareLaunchArgument(
                "print_positions",
                default_value="false",
                description="Log object positions (x,y,z) to console instead of displaying image",
            ),
            Node(
                package="perception",
                executable="aligned_depth_publisher",
                name="aligned_depth_publisher",
                prefix=VENV_PYTHON,
                parameters=[
                    global_params,
                    {"camera_type": LaunchConfiguration("camera_type")},
                    {"camera_key": LaunchConfiguration("camera_key")},
                ],
            ),
            Node(
                package="perception",
                executable="object_localizer",
                name="object_localizer",
                prefix=VENV_PYTHON,
                parameters=[
                    global_params,
                    {"model_name": LaunchConfiguration("model_name")},
                    {"object_id": LaunchConfiguration("object_id")},
                    {"camera_key": LaunchConfiguration("camera_key")},
                    {"visualize_camera": LaunchConfiguration("visualize_camera")},
                    {"print_positions": LaunchConfiguration("print_positions")},
                ],
            ),
        ]
    )
