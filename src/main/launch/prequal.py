from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")

hardware_launch_path = os.path.join(os.getcwd(), "src/main/launch/hardware.py")
localization_launch_path = os.path.join(os.getcwd(), "src/main/launch/localization.py")

def generate_launch_description():
    # Include the other launch files
    prequal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hardware_launch_path)
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path)
    )

    Node(
        package="control",
        executable="controller",
        parameters=[global_params, {"velocity_only": False}],
    ),

    Node(
        package="control",
        executable="prequal",
        parameters=[global_params],
    ),

    # Launch them with delays if desired
    return LaunchDescription([
        TimerAction(period=0.0, actions=[prequal_launch]),
        TimerAction(period=5.0, actions=[localization_launch]),
    ])