from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")
### the robot_localization launch here can be deleted

def generate_launch_description():

    """
    Generate launch description for localization launch.

    This function generates a LaunchDescription object which
    contains two Node objects: one for the "hardware" package
    and "sensors" executable, and one for the "robot_localization"
    package and "ekf_node" executable. Both nodes are configured
    with the global parameters.

    Returns:
        LaunchDescription: a launch description object
    """
    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="sensors",
                name="sensors",
                parameters=[global_params],
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[global_params],
            ),
        ]
    )
