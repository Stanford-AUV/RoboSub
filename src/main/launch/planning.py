from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Which waypoint yaml to run (bare filename resolves against the
            # planning package share dir). Set from launch_sub.sh:
            #   ./launch_sub.sh [waypoints.yaml]
            DeclareLaunchArgument(
                "waypoints_path",
                default_value="segments.yaml",
                description="Waypoint yaml for path_generator (must be baked "
                "first: python tools/bake_path.py <yaml>)",
            ),
            # Run the hydrophone pinger node (publishes /pinger/task from the
            # Daisy boards' front/back verdict). Only for pinger missions --
            # it holds both Daisy serial ports open. launch_sub.sh enables it
            # automatically when the waypoints yaml name contains "pinger".
            DeclareLaunchArgument(
                "pinger",
                default_value="false",
                description="Launch the hydrophone pinger task-order node",
            ),
            Node(
                package="hardware",
                executable="pinger",
                output="screen",
                condition=IfCondition(LaunchConfiguration("pinger")),
            ),
            Node(
                package="planning",
                executable="path_generator",
                parameters=[
                    {"waypoints_path": LaunchConfiguration("waypoints_path")}
                ],
                # path_generator exits itself ~3s after the path finishes;
                # shut down this whole launch when it does so launch_sub.sh's
                # `wait -n` fires its full-stack teardown (SIGINT + neutral PWM).
                on_exit=Shutdown(),
            ),
            Node(
                package="planning",
                executable="path_streamer",
            ),
        ]
    )
