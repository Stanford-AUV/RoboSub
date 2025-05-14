from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(depthai_prefix, 'launch', 'camera.launch.py')
                )
            ),
            Node(
                package="perception",
                executable="camera_viewer",
                output="screen",
                parameters=[],
            ),
            # Node(
            #     package="perception",
            #     executable="camera",
            #     output="screen",
            #     parameters=[],
            # ),
            # Node(
            #     package="perception",
            #     executable="test.view_video",
            #     output="screen",
            #     parameters=[],
            # ),
        ]
    )
