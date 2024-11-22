from launch import LaunchDescription
from launch_ros.actions import Node
import os

global_params = os.path.join(
    os.path.dirname(__file__), "..", "..", "launch", "params", "global.yaml"
)


def generate_launch_description():
    timer_period = 1.0  # Slow down timer period for debugging
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="thrust_generator",
                parameters=[
                    global_params,
                    {"timer_period": timer_period},
                ],
            ),
            Node(
                package="hardware",
                executable="thrusters",
                parameters=[
                    global_params,
                    {"timer_period": timer_period},
                ],
            ),
        ]
    )
