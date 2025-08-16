# frames_and_mission.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # NOTE: static_transform_publisher expects:
    #   x y z roll pitch yaw parent_frame child_frame

    front_cam_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="front_cam_to_base",
        output="screen",
        arguments=[
            "0.20", "0.0", "0.10",   # x y z (m)
            "0", "0", "0",           # roll pitch yaw (rad)
            "base_link", "camera_front_optical_frame",
        ],
    )

    bottom_cam_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bottom_cam_to_base",
        output="screen",
        arguments=[
            "-0.066", "0.0", "0.0",  # x y z (m)
            "0", "1.5708", "0",      # roll pitch yaw (rad)
            "base_link", "camera_bottom_optical_frame",
        ],
    )

    # --- your nodes ---
    objects_localizer = Node(
        package="perception",
        executable="objects_localizer",
        name="objects_localizer",
        output="screen",
    )

    torpedo_task = Node(
        package="planning",
        executable="torpedo_task",
        name="torpedo_task",
        output="screen",
    )

    track_object = Node(
        package="planning",
        executable="track_object",
        name="track_object",
        output="screen",
    )

    return LaunchDescription([
        front_cam_to_base,
        bottom_cam_to_base,
        objects_localizer,
        torpedo_task,
        track_object,
    ])
