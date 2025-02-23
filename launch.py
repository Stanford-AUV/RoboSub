import os
import launch
import launch.actions
import launch_ros.actions


def generate_launch_description():
    log_dir = "/tmp/ros2_fzf_logs"
    os.makedirs(log_dir, exist_ok=True)  # Ensure log directory exists

    # TODO
    node_actions = [
        launch_ros.actions.Node(
            package="control",
            executable="thrust_generator",
            name="thrust_generator",
            output="log",
            emulate_tty=True,
        ),
        launch_ros.actions.Node(
            package="control",
            executable="controller",
            name="controller",
            output="log",
            emulate_tty=True,
        ),
    ]

    return launch.LaunchDescription(node_actions)
