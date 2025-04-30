from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="perception",
                executable="objects_localizer",
                output="screen",
                parameters=[
                    {"view_detections": True},
                ],
            ),
            Node(
                package="perception",
                executable="test.view_detections_3d",
                output="screen",
                parameters=[],
            ),
            Node(
                package="perception",
                executable="camera",
                output="screen",
                parameters=[
                    {"view_detections": True},
                    {"view_detections_3d": True},
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "/oak/rgb/image_raw",
                    "/oak/depth/image_raw",
                    "--output",
                    "rosbag2_oak_camera",
                ],
                output="screen",
            ),
        ]
    )
