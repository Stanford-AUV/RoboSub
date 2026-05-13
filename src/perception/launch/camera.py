import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

global_params = os.path.join(
    os.path.dirname(__file__), "..", "..", "main", "launch", "params", "global.yaml"
)

def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_viewer",
                default_value="false",
                description="Launch the camera_viewer node",
            ),
            DeclareLaunchArgument(
                "camera_names",
                default_value="oak_0",
                description="Comma-separated camera names for camera_viewer",
            ),
            DeclareLaunchArgument(
                "photographer",
                default_value="false",
                description="Launch the photographer node (saves periodic snapshots to disk)",
            ),
            DeclareLaunchArgument(
                "photographer_cameras",
                default_value="oak_0",
                description="Comma-separated camera names for the photographer",
            ),
            DeclareLaunchArgument(
                "photographer_output_dir",
                default_value="real/wet_data",
                description="Directory where the photographer saves captured frames",
            ),
            DeclareLaunchArgument(
                "photographer_period",
                default_value="0.25",
                description="Seconds between photographer snapshots",
            ),
            Node(
                package="perception",
                executable="oak_node",
                parameters=[
                    global_params,
                ],
            ),
            Node(
                package="perception",
                executable="realsense_node",
                parameters=[
                    global_params,
                ],
            ),
            OpaqueFunction(function=_launch_camera_viewers),
            OpaqueFunction(function=_launch_photographer),
        ]
    )


def _launch_camera_viewers(context, *args, **kwargs):
    if not IfCondition(LaunchConfiguration("camera_viewer")).evaluate(context):
        return []

    raw_names = LaunchConfiguration("camera_names").perform(context)
    names = [name.strip() for name in raw_names.split(",") if name.strip()]

    return [
        Node(
            package="perception",
            executable="camera_viewer",
            arguments=[name],
            parameters=[
                global_params,
            ],
        )
        for name in names
    ]


def _launch_photographer(context, *args, **kwargs):
    if not IfCondition(LaunchConfiguration("photographer")).evaluate(context):
        return []

    raw_names = LaunchConfiguration("photographer_cameras").perform(context)
    names = [name.strip() for name in raw_names.split(",") if name.strip()]
    if not names:
        return []

    output_dir = LaunchConfiguration("photographer_output_dir").perform(context)
    period = LaunchConfiguration("photographer_period").perform(context)

    return [
        Node(
            package="perception",
            executable="photographer",
            arguments=names,
            parameters=[
                global_params,
                {
                    "output_dir": output_dir,
                    "capture_period_s": float(period),
                },
            ],
        )
    ]