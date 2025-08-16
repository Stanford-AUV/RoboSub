# save as src/main/launch/full_stack.launch.py

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Step 1: include your existing localization launch
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('main'),  # package name where localization.py lives
                'launch',
                'localization.py'
            ])
        ])
    )

    # Step 2: controller node
    controller_node = Node(
        package='control',
        executable='controller',
        output='screen'
    )

    # Step 3: path tracker node
    path_tracker_node = Node(
        package='control',
        executable='path_tracker',
        output='screen'
    )

    return LaunchDescription([
        # Start localization immediately
        localization_launch,

        # After 5 seconds, start controller
        TimerAction(period=1.0, actions=[controller_node]),

        # After another 5 seconds (10 total), start path_tracker
        TimerAction(period=2.0, actions=[path_tracker_node]),
    ])
