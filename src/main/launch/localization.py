from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# global_params = os.path.join(os.path.dirname(__file__), "params", "global.yaml")
### the robot_localization launch here can be deleted
ekf_params = os.path.join(os.path.dirname(__file__), "..", "..", "localization", "localization", "config", "ekf.yaml")

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

    # ekf_config = os.path.join(get_package_share_directory('localization'), 'ekf.yaml')
    
    return LaunchDescription(
        [
            # Node(
            #     package="hardware",
            #     executable="sensors",
            #     name="sensors",
            #     parameters=[global_params],
            # ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_imu_tf",
                arguments=[
                    "-0.1525", "-0.02", "0.1375",   # x y z
                    "-1.57079632679", "0.0", "1.57079632679", # roll pitch yaw (rad)
                    "base_link",
                    "imu_frame",
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_dvl_tf",
                arguments=[
                    "-0.105", "0.0", "-0.0625",     # x y z
                    "0.0", "0.0", "0.0",             # roll pitch yaw (rad)
                    "base_link",
                    "dvl_frame",
                ],
            ),

        ]
    )
