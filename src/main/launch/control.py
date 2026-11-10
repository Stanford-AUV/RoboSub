from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # When included from main.py, hardware.py already runs thrust_generator.
    # For `ros2 launch main control.py` without hardware, add thrust_generator
    # here or run: ros2 run control thrust_generator (it reads
    # hardware/thrusters.yaml itself).
    return LaunchDescription(
        [
            Node(
                package="control",
                executable="pid_control",
            ),
        ]
    )
