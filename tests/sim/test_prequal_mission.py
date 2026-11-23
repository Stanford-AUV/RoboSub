"""Sim mission test: run prequal.yaml in the full sim stack and check it works end to end.

Run in the container from /ws (after colcon build and source install/setup.bash):
    launch_test tests/sim/test_prequal_mission.py
Lives outside src/ on purpose: colcon test (./test.sh) only runs package test/ folders,
so this slow, Gazebo-needing test runs only in its own CI job.
"""
import os
import unittest

import launch
import launch_testing.actions
import pytest
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM_STACK = os.path.join(REPO, "src", "main", "launch", "sim_stack.py")

# Temporary: the sim runs well below real time on the Mac (amd64 emulation).
# Lower this again once the sim is faster (see the plan's "Keep the sim fast enough").
MISSION_TIMEOUT_S = 900


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIM_STACK),
        launch_arguments={"mission": "prequal.yaml", "yaw_drift_deg_per_s": "0.0",
                          "cameras": "false"}.items(),
    )
    # ReadyToTest: start the test methods below now, while the sim keeps running.
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestMission(unittest.TestCase):
    def test_mission_finishes(self, proc_output):
        # path_generator logs this after the path and its 3 s hold, just before it
        # exits; its exit then shuts the whole launch down (planning.py on_exit=Shutdown).
        proc_output.assertWaitFor("Hold elapsed", timeout=MISSION_TIMEOUT_S)