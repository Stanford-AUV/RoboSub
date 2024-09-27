from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    timer_period = 1.0  # Slow down timer period for debugging
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrusters",
                parameters=["params/thrusters.yaml", {"timer_period": timer_period}],
            ),
            Node(
                package="hardware",
                executable="thrusters",
                parameters=["params/thrusters.yaml", {"timer_period": timer_period}],
            ),
        ]
    )
