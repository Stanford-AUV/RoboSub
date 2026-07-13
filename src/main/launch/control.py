from launch import LaunchDescription
from launch_ros.actions import Node
import os



def generate_launch_description():
    # When included from main.py, hardware.py already runs thrust_generator.
    # For `ros2 launch main control.py` without hardware, add thrust_generator here
    # or run: ros2 run control thrust_generator (it reads hardware/thrusters.yaml itself)
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="pid_control",
                parameters=[{"velocity_only": False}],
            ),
            # Publishes sample waypoints on topic `waypoint` for the controller.
            # # Swap for path_tracker + a get_waypoints server when running full missions.
            # Node(
            #     package="control",
            #     executable="test_controller",
            # ),
            # START B — path following via service (needs a get_waypoints server; see README)
            # Node(
            #     package="control",
            #     executable="path_tracker",
            # ),
            # END B
            Node(
                package="control",
                executable="logger",
            ),
        ]
    )
