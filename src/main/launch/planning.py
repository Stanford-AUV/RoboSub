from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import DeclareLaunchArgument, Shutdown


global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="planning",
                executable="path_generator",
                parameters=[global_params],
                # path_generator exits itself ~3s after the path finishes;
                # shut down this whole launch when it does so launch_sub.sh's
                # `wait -n` fires its full-stack teardown (SIGINT + neutral PWM).
                on_exit=Shutdown(),
            ),
            Node(
                package="planning",
                executable="path_streamer",
                parameters=[global_params],
            ),
        ]
    )
