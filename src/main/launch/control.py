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
                parameters=[global_params, {"velocity_only": False}],
            ),
            # Publishes sample waypoints on topic `waypoint` for the controller.
            # Swap for path_tracker + a get_waypoints server when running full missions.
            Node(
                package="control",
                executable="test_controller",
                parameters=[global_params],
            ),
            # START B — path following via service (needs a get_waypoints server; see README)
            # Node(
            #     package="control",
            #     executable="path_tracker",
            #     parameters=[global_params],
            # ),
            # END B
            # Node(
            #     package="control",
            #     executable="logger",
            #     parameters=[global_params],
            # ),
        ]
    )
