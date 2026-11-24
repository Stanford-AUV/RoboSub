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
import csv
import os
import threading
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import WrenchStamped
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM_STACK = os.path.join(REPO, "src", "main", "launch", "sim_stack.py")
# Every recorded sample, for plotting a failed run. CI uploads this file.
OUT_CSV = os.path.join(REPO, "tests", "sim", "out", "prequal.csv")

# Temporary: the sim runs well below real time on the Mac (amd64 emulation).
# Lower this again once the sim is faster (see the plan's "Keep the sim fast enough").
MISSION_TIMEOUT_S = 900

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


def _sample(msg):
    """(t, x, y, z): t is the message's own stamp, which is sim time on both topics."""
    p = msg.pose.pose.position
    return (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, p.x, p.y, p.z)


class Recorder(Node):
    """Stores what the stack publishes, for grading after the sim stops."""

    def __init__(self):
        super().__init__("mission_test_recorder")
        self.truth = []
        self.ekf = []
        self.wrench_count = 0
        self.create_subscription(Odometry, "/odometry/ground_truth",
                                 lambda m: self.truth.append(_sample(m)), 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda m: self.ekf.append(_sample(m)), 10)
        self.create_subscription(WrenchStamped, "/wrench", self._on_wrench, 10)

    def _on_wrench(self, _msg):
        self.wrench_count += 1


class TestMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Recorder()
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        # Spin on a background thread: the main thread is busy in assertWaitFor below.
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=5)
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
        os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
        with open(OUT_CSV, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["source", "t", "x", "y", "z"])
            w.writerows(("truth",) + s for s in cls.truth)
            w.writerows(("ekf",) + s for s in cls.ekf)

    def test_topics_published(self):
        self.assertGreater(len(self.truth), 0,
                           "no /odometry/ground_truth (Gazebo, bridge or ground_truth node down)")
        self.assertGreater(len(self.ekf), 0,
                           "no /odometry/filtered (EKF or its sensor inputs down)")
        self.assertGreater(self.wrench_count, 0,
                           "no /wrench (pid_control down, or it never got a target)")

    def test_sub_dove(self):
        self.assertTrue(self.truth, "no ground truth to grade")
        lowest = min(z for _, _, _, z in self.truth)
        self.assertLess(lowest, DIVE_BELOW_Z,
                        f"deepest true z was {lowest:.2f} m; expected below {DIVE_BELOW_Z} m")

    def test_sub_moved_forward(self):
        self.assertTrue(self.truth, "no ground truth to grade")
        furthest = max(x for _, x, _, _ in self.truth)
        self.assertGreater(furthest, FORWARD_PAST_X,
                           f"furthest true x was {furthest:.2f} m; expected past {FORWARD_PAST_X} m")

    def test_ekf_matches_truth_on_first_leg(self):
        hit = next(((t, x, y) for t, x, y, _ in self.truth if x >= EKF_CHECK_AT_X), None)
        self.assertIsNotNone(hit, f"sub never reached x = {EKF_CHECK_AT_X} m")
        self.assertTrue(self.ekf, "no EKF output to compare")
        t, tx, ty = hit
        et, ex, ey, _ = min(self.ekf, key=lambda s: abs(s[0] - t))
        self.assertLess(abs(et - t), MAX_STAMP_GAP_S,
                        f"no EKF sample near t = {t:.2f} s (nearest at {et:.2f} s)")
        self.assertLess(abs(ex - tx), EKF_TOLERANCE_M,
                        f"at t = {t:.1f} s: EKF x {ex:.3f} vs truth {tx:.3f}")
        self.assertLess(abs(ey - ty), EKF_TOLERANCE_M,
                        f"at t = {t:.1f} s: EKF y {ey:.3f} vs truth {ty:.3f}")