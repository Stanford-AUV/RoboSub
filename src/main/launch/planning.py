from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


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
            # path_streamer redraws a 3D plot of the whole desired path on every
            # /desired/pose message (60 Hz) and took ~3 CPU cores in sim (Nov 23),
            # so sim_stack.py turns it off. The sub keeps it (default true).
            DeclareLaunchArgument("path_plot", default_value="true"),
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
                condition=IfCondition(LaunchConfiguration("path_plot")),
            ),
        ]
    )
