from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrust_generator",
                parameters=[global_params],
            ),
            Node(
                package="control",
                executable="controller",
                parameters=[global_params],
            ),
            # Run either A or B
            # START A
            # Node(
            #     package="control",
            #     executable="test_controller",
            #     parameters=[global_params],
            # ),
            # END A
            # START B
            # Path generator currently disabled, but would need to be inserted here
            Node(
                package="control",
                executable="test_waypoints",
                parameters=[global_params],
            ),
            Node(
                package="control",
                executable="path_tracker",
                parameters=[global_params],
            ),
            # END B
            Node(
                package="control",
                executable="logger",
                parameters=[global_params],
            ),
        ]
    )
