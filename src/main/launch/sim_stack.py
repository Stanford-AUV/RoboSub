"""Bring up the whole stack in sim, the way launch_sub.sh does on the sub.

  ros2 launch src/main/launch/sim_stack.py                                 # prequal.yaml
  ros2 launch src/main/launch/sim_stack.py mission:=segments.yaml
  ros2 launch src/main/launch/sim_stack.py control:=false planning:=false  # manual /wrench tests
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

# Paths relative to this file, so it runs straight from src/ with no rebuild.
HERE = os.path.dirname(os.path.realpath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
SIM = os.path.join(REPO, "src", "simulation", "simulation")
MISSIONS = os.path.join(REPO, "src", "planning", "planning")


def include(name, **kwargs):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(HERE, name)), **kwargs)


def check_build_is_fresh():
    """Refuse to start if a .py under src/ is newer than the build (launch_sub.sh does this too)."""
    built = os.path.getmtime(os.path.join(REPO, "install", "setup.bash"))
    for root, _, files in os.walk(os.path.join(REPO, "src")):
        if "/launch" in root:
            continue
        for f in files:
            path = os.path.join(root, f)
            if f.endswith(".py") and os.path.getmtime(path) > built:
                raise RuntimeError(f"{path} is newer than the last build; run colcon build first")


def generate_launch_description():
    check_build_is_fresh()
    mission_file = PathJoinSubstitution([MISSIONS, LaunchConfiguration("mission")])

    bake = ExecuteProcess(cmd=["python3", os.path.join(REPO, "tools", "bake_path.py"), mission_file],
                          output="screen")

    return LaunchDescription([
        DeclareLaunchArgument("mission", default_value="prequal.yaml"),
        DeclareLaunchArgument("control", default_value="true"),
        DeclareLaunchArgument("planning", default_value="true"),

        # Same as the sub: non-blocking publishing.
        SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE",
                               os.path.join(REPO, "config", "fastdds_async.xml")),
        # Every node started after this runs on Gazebo's clock.
        SetParameter(name="use_sim_time", value=True),

        # 1. The world, and the bridge into ROS.
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", "--headless-rendering",
                            os.path.join(SIM, "models", "world.urdf")], output="screen"),
        Node(package="ros_gz_bridge", executable="parameter_bridge",
             parameters=[{"config_file": os.path.join(SIM, "gazebo_bridge.yaml")}]),

        # 2. After Gazebo is up: the sim stand-ins, the real processing nodes, the unchanged stack.
        TimerAction(period=3.0, actions=[
            include("simulation.py"),
            Node(package="hardware", executable="imu"),
            Node(package="hardware", executable="depth"),
            Node(package="control", executable="thrust_generator"),
            include("localization.py"),
            include("control.py", condition=IfCondition(LaunchConfiguration("control"))),
        ]),

        # 3. Bake the mission, then start planning once the stack has settled.
        GroupAction(condition=IfCondition(LaunchConfiguration("planning")), actions=[
            bake,
            RegisterEventHandler(OnProcessExit(target_action=bake, on_exit=[
                TimerAction(period=8.0, actions=[
                    include("planning.py",
                            launch_arguments={"waypoints_path": mission_file}.items()),
                ]),
            ])),
        ]),
    ])