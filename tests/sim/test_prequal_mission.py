"""Sim mission test: run prequal.yaml in the full sim stack and check it works end to end.

Run in the container from /ws (after colcon build and source install/setup.bash):
    launch_test tests/sim/test_prequal_mission.py
Lives outside src/ on purpose: colcon test (./test.sh) only runs package test/ folders,
so this slow, Gazebo-needing test runs only in its own CI job.

Two phases:
  TestMission (active): runs while the sim is up. Records the topics below and waits for
      the mission to finish.
  TestGrade (post-shutdown): runs after the sim has stopped. Grades the recording.
      Grading happens here, not in TestMission, because path_generator shuts the whole
      launch down right after its last log line; checks still running in the active
      phase would be cut off ("Processes under test stopped before tests completed").

Plumbing-level thresholds: they catch a broken stack, not a badly tuned one. Tighten
them (gate pass, depth tolerance) once Phase 2 makes the sim physics trustworthy.
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from sim_helpers import SIM_STACK, Recorder, Spinner, write_samples_csv  # noqa: E402

# Wall-clock limit for the whole mission: about 3x its slowest measured run (GitHub's 2-core
# runner took 213 s, the Mac 186 s, Nov 23), so a stuck mission fails within minutes.
MISSION_TIMEOUT_S = 600

DIVE_BELOW_Z = -0.3      # m; prequal dives to -0.6
FORWARD_PAST_X = 2.0     # m; prequal's straight first leg ends at x = 2.3
EKF_CHECK_AT_X = 2.2     # m; still on the straight leg, before the spins (heading 0)
EKF_TOLERANCE_M = 0.1    # m; every run so far agreed to about 0.01 m here
MAX_STAMP_GAP_S = 0.1    # s; EKF and truth samples compared must be this close in time

# Filled by TestMission.tearDownClass, read by TestGrade (same module, same process).
RECORDED = {}


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIM_STACK),
        launch_arguments={"mission": "prequal.yaml", "yaw_drift_deg_per_s": "0.0",
                          "cameras": "false"}.items(),
    )
    # ReadyToTest: start the active tests now, while the sim keeps running.
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


class TestMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Recorder("mission_test_recorder")
        # Spin on a background thread: the main thread is busy in assertWaitFor below.
        cls.spinner = Spinner(cls.node)

    @classmethod
    def tearDownClass(cls):
        cls.spinner.stop()
        RECORDED.update(truth=list(cls.node.truth), ekf=list(cls.node.ekf),
                        wrench_count=cls.node.wrench_count)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_mission_finishes(self, proc_output):
        # path_generator logs this after the path and its 3 s hold, just before it exits.
        proc_output.assertWaitFor("Hold elapsed", timeout=MISSION_TIMEOUT_S)


@launch_testing.post_shutdown_test()
class TestGrade(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.truth = RECORDED.get("truth", [])
        cls.ekf = RECORDED.get("ekf", [])
        cls.wrench_count = RECORDED.get("wrench_count", 0)
        write_samples_csv("prequal.csv", truth=cls.truth, ekf=cls.ekf)

    def test_topics_published(self):
        self.assertGreater(len(self.truth), 0,
                           "no /odometry/ground_truth (Gazebo, bridge or ground_truth node down)")
        self.assertGreater(len(self.ekf), 0,
                           "no /odometry/filtered (EKF or its sensor inputs down)")
        self.assertGreater(self.wrench_count, 0,
                           "no /wrench (pid_control down, or it never got a target)")

    def test_sub_dove(self):
        self.assertTrue(self.truth, "no ground truth to grade")
        lowest = min(s["z"] for s in self.truth)
        self.assertLess(lowest, DIVE_BELOW_Z,
                        f"deepest true z was {lowest:.2f} m; expected below {DIVE_BELOW_Z} m")

    def test_sub_moved_forward(self):
        self.assertTrue(self.truth, "no ground truth to grade")
        furthest = max(s["x"] for s in self.truth)
        self.assertGreater(furthest, FORWARD_PAST_X,
                           f"furthest true x was {furthest:.2f} m; expected past {FORWARD_PAST_X} m")

    def test_ekf_matches_truth_on_first_leg(self):
        hit = next((s for s in self.truth if s["x"] >= EKF_CHECK_AT_X), None)
        self.assertIsNotNone(hit, f"sub never reached x = {EKF_CHECK_AT_X} m")
        self.assertTrue(self.ekf, "no EKF output to compare")
        nearest = min(self.ekf, key=lambda s: abs(s["t"] - hit["t"]))
        t = hit["t"]
        self.assertLess(abs(nearest["t"] - t), MAX_STAMP_GAP_S,
                        f"no EKF sample near t = {t:.2f} s (nearest at {nearest['t']:.2f} s)")
        self.assertLess(abs(nearest["x"] - hit["x"]), EKF_TOLERANCE_M,
                        f"at t = {t:.1f} s: EKF x {nearest['x']:.3f} vs truth {hit['x']:.3f}")
        self.assertLess(abs(nearest["y"] - hit["y"]), EKF_TOLERANCE_M,
                        f"at t = {t:.1f} s: EKF y {nearest['y']:.3f} vs truth {hit['y']:.3f}")
