"""Thruster direction test: does each /wrench axis move the sim sub the right way?

Run in the container from /ws (after colcon build and source install/setup.bash):
    launch_test tests/sim/test_thruster_directions.py

The stack runs with the controller and planner off, so this test is the only /wrench
publisher. It sends one axis at a time and grades the direction of the response from
ground truth. The real thrust_generator and thrusters.yaml do the allocation, so a sign
flip or swapped thruster in thrusters.yaml, a sign bug in the allocator, or a wrong joint
axis or bridge topic in the sim model fails here instead of on a pool day. It cannot catch
a thruster wired backwards on the real sub: that is tools/wrench_sweep.py's job on the bench.

Units: /wrench forces are kgf and torques kgf·m (thrust_generator feeds the T200 kgf curve).

Each axis: brake until the sub is still, then a PULSE_S pulse on that axis.
  Brake: the test pushes against every velocity and rotation rate it reads from ground
    truth. The sim's rotation is barely damped (it swings for many seconds after a twist,
    and a turn started by one pulse never stops on its own), so without it each axis would
    start in the wobble of the one before. This uses ground truth only to put the sub
    back in a still starting position, like an operator between tests; the stack is
    still only judged on what it does with its own sensors.
  Pulse: the tested axis is free; every other axis stays braked so it does not pick up
    side effects (for example, sideways motion while still drifting forward turns an
    elongated hull).
Grading: the change from the mean over the last GRADE_WINDOW_S before the pulse.
  Velocities rise steadily, so they are graded on their mean over the last
    GRADE_WINDOW_S of the pulse.
  Angles swing (the first swing of a step torque always goes the commanded way, then the
    sub swings back), so they are graded on their peak change in the commanded direction.

Order: Fx and Fy at the surface (buoyancy holds the depth there), then a -Fz dive (graded:
the sub must go down), then Fz, Mx, My, Mz at depth, where every message also carries
BIAS_FZ_KGF to cancel the sim's net buoyancy.

Two phases, like test_prequal_mission.py: TestSequence (active) drives the sub while the
sim runs; TestDirections (post-shutdown) grades each axis as its own test, so a failure
names the axis.
"""
import csv
import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import WrenchStamped  # noqa: E402
from launch.actions import IncludeLaunchDescription  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from sim_helpers import OUT_DIR, SIM_STACK, Recorder, Spinner, write_samples_csv  # noqa: E402

FORCE_KGF = 0.5      # well under thrust_generator's clip (4 kgf); about 0.15-0.3 m/s in 3 s
TORQUE_KGFM = 0.1    # under the clip (1 kgf·m); first swing about 10-17° (Nov 24 run)
DIVE_FZ_KGF = -1.0
# model.urdf: 26.80 kg of mass against 27.4 kg of displaced water (hull plus thruster boxes),
# so about 0.6 kgf of net buoyancy. Change this if the mass or the collision boxes change.
BIAS_FZ_KGF = -0.6

# Brake (test harness only): wrench = -gain x ground-truth velocity / rotation rate, body frame.
BRAKE_KGF_PER_MPS = 2.0      # 0.3 m/s -> 0.6 kgf
BRAKE_KGFM_PER_RADPS = 0.15  # 40°/s spin -> 0.1 kgf·m
BRAKE_LIMITS = [1.0, 1.0, 1.0, 0.2, 0.2, 0.2]   # kgf, kgf·m; clip so the brake stays gentle
STILL_MPS = 0.03             # "still" means every velocity below this ...
STILL_RADPS = 0.05           # ... and every rotation rate below this (about 3°/s) ...
STILL_FOR_S = 1.0            # ... continuously for this long
SETTLE_MIN_S = 2.0
SETTLE_MAX_S = 25.0          # carry on (and note it) if the sub never quite settles

SURFACE_SETTLE_S = 10.0      # the sub spawns at z = 0 and bobs at the surface for a few seconds
DIVE_S = 3.0
PULSE_S = 3.0
GRADE_WINDOW_S = 1.0

PUBLISH_PERIOD_S = 0.05  # wall time between /wrench messages (thrust_generator also repeats the last one)
STARTUP_TIMEOUT_S = 120  # wall time for ground truth to start flowing
WALL_PER_SIM_S = 20      # wall seconds allowed per sim second (the Mac sim runs near 0.5x)
FLOOR_GUARD_Z = -2.5     # m; the pool floor is at -3.0 (world.urdf), so stop before reaching it

# name: (wrench index [Fx, Fy, Fz, Mx, My, Mz], magnitude, graded field, min change, units,
#        grading mode, expected motion; wording from tools/wrench_sweep.py's EXPECT table)
AXES = {
    "Fx": (0, FORCE_KGF, "vx", 0.05, "m/s", "end", "surge forward"),
    "Fy": (1, FORCE_KGF, "vy", 0.05, "m/s", "end", "sway left / to port"),
    "Fz": (2, FORCE_KGF, "vz", 0.05, "m/s", "end", "heave up"),
    "Mx": (3, TORQUE_KGFM, "roll", 2.0, "deg", "peak", "roll: port side rises / starboard dips"),
    "My": (4, TORQUE_KGFM, "pitch", 2.0, "deg", "peak", "pitch: nose down"),
    "Mz": (5, TORQUE_KGFM, "yaw", 5.0, "deg", "peak", "yaw left / counter-clockwise from above"),
}
SURFACE_AXES = ["Fx", "Fy"]
DEPTH_AXES = ["Fz", "Mx", "My", "Mz"]
DIVE_MIN_DROP_M = 0.05
VELOCITY_FIELDS = ["vx", "vy", "vz"]
RATE_FIELDS = ["wx", "wy", "wz"]

# Filled by TestSequence, read by TestDirections (same module, same process).
RECORDED = {"pulses": {}, "settled": {}}


@pytest.mark.launch_test
def generate_test_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(SIM_STACK),
        launch_arguments={"control": "false", "planning": "false", "cameras": "false",
                          "yaw_drift_deg_per_s": "0.0"}.items(),
    )
    return launch.LaunchDescription([sim, launch_testing.actions.ReadyToTest()])


def wrench_msg(node, values):
    """WrenchStamped from [Fx, Fy, Fz, Mx, My, Mz] (kgf, kgf·m), frame base_link."""
    msg = WrenchStamped()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"
    f, m = msg.wrench.force, msg.wrench.torque
    f.x, f.y, f.z, m.x, m.y, m.z = (float(v) for v in values)
    return msg


def brake_wrench(sample, bias_fz, free_index=None):
    """Push against every body-frame velocity and rotation rate in `sample`, except the axis
    `free_index` (the one being tested), plus the depth bias on Fz."""
    rates = [sample[f] for f in VELOCITY_FIELDS] + [sample[f] for f in RATE_FIELDS]
    gains = [BRAKE_KGF_PER_MPS] * 3 + [BRAKE_KGFM_PER_RADPS] * 3
    values = [float(np.clip(-g * r, -lim, lim)) for g, r, lim in zip(gains, rates, BRAKE_LIMITS)]
    if free_index is not None:
        values[free_index] = 0.0
    values[2] += bias_fz
    return values


def is_still(sample):
    return (all(abs(sample[f]) < STILL_MPS for f in VELOCITY_FIELDS)
            and all(abs(sample[f]) < STILL_RADPS for f in RATE_FIELDS))


class TestSequence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Recorder("thruster_direction_test")
        cls.pub = cls.node.create_publisher(WrenchStamped, "/wrench", 10)
        cls.spinner = Spinner(cls.node)

    @classmethod
    def tearDownClass(cls):
        # Leave the thrusters at zero: thrust_generator keeps repeating the last wrench.
        for _ in range(10):
            cls.pub.publish(wrench_msg(cls.node, [0.0] * 6))
            time.sleep(PUBLISH_PERIOD_S)
        cls.spinner.stop()
        RECORDED["truth"] = list(cls.node.truth)
        cls.node.destroy_node()
        rclpy.shutdown()

    def latest(self):
        return self.node.truth[-1]

    def run_for(self, wrench_fn, sim_seconds, what, stop_when=None):
        """Publish wrench_fn() until `sim_seconds` of sim time pass, or until stop_when()
        returns True. Returns (start, end) in sim time."""
        start = self.node.sim_now()
        deadline = time.monotonic() + sim_seconds * WALL_PER_SIM_S + 30
        while self.node.sim_now() - start < sim_seconds:
            self.pub.publish(wrench_msg(self.node, wrench_fn()))
            if self.latest()["z"] < FLOOR_GUARD_Z:
                self.fail(f"sub sank to z = {self.latest()['z']:.2f} m during {what}; "
                          f"is BIAS_FZ_KGF ({BIAS_FZ_KGF}) still the sim's net buoyancy?")
            if time.monotonic() > deadline:
                self.fail(f"sim time stopped advancing during {what} "
                          f"(only {self.node.sim_now() - start:.1f} of {sim_seconds} s)")
            if stop_when is not None and stop_when():
                break
            time.sleep(PUBLISH_PERIOD_S)
        return start, self.node.sim_now()

    def settle(self, bias_fz, what):
        """Brake until the sub has been still for STILL_FOR_S (at least SETTLE_MIN_S,
        at most SETTLE_MAX_S). Returns True if it got still."""
        state = {"still_since": None}
        start = self.node.sim_now()

        def settled():
            now = self.node.sim_now()
            if not is_still(self.latest()):
                state["still_since"] = None
                return False
            if state["still_since"] is None:
                state["still_since"] = now
            return now - start >= SETTLE_MIN_S and now - state["still_since"] >= STILL_FOR_S

        self.run_for(lambda: brake_wrench(self.latest(), bias_fz), SETTLE_MAX_S,
                     f"settle before {what}", stop_when=settled)
        return settled()

    def pulse(self, name, bias_fz):
        index, magnitude = AXES[name][0], AXES[name][1]
        RECORDED["settled"][name] = self.settle(bias_fz, name)

        def pulse_wrench():
            values = brake_wrench(self.latest(), bias_fz, free_index=index)
            values[index] += magnitude
            return values

        RECORDED["pulses"][name] = self.run_for(pulse_wrench, PULSE_S, f"{name} pulse")

    def test_run_sequence(self):
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while not self.node.truth or self.node.sim_now() == 0.0:
            self.assertLess(time.monotonic(), deadline,
                            "no /odometry/ground_truth or /clock (Gazebo, bridge or ground_truth down)")
            time.sleep(0.1)

        self.run_for(lambda: brake_wrench(self.latest(), 0.0), SURFACE_SETTLE_S, "surface settle")
        for name in SURFACE_AXES:
            self.pulse(name, bias_fz=0.0)

        self.settle(0.0, "dive")

        def dive_wrench():
            values = brake_wrench(self.latest(), 0.0, free_index=2)
            values[2] += DIVE_FZ_KGF
            return values

        RECORDED["dive"] = self.run_for(dive_wrench, DIVE_S, "dive")
        for name in DEPTH_AXES:
            self.pulse(name, bias_fz=BIAS_FZ_KGF)


def response(truth, field, start, end, mode):
    """Change in `field` during the window [start, end] relative to its mean over the
    GRADE_WINDOW_S before `start`.
    mode "end": mean over the last GRADE_WINDOW_S of the window, minus that baseline.
    mode "peak": the largest value in the window, minus that baseline (for angles, whose
        first swing goes the commanded way before the sub swings back).
    Angles are unwrapped first, so a yaw that crosses ±180° counts as the turn it is."""
    samples = [s for s in truth if start - GRADE_WINDOW_S <= s["t"] <= end]
    if not samples:
        return None
    t = np.array([s["t"] for s in samples])
    v = np.array([s[field] for s in samples])
    if field in ("roll", "pitch", "yaw"):
        v = np.degrees(np.unwrap(np.radians(v)))
    before = v[t < start]
    during = v[t >= start]
    if before.size == 0 or during.size == 0:
        return None
    baseline = before.mean()
    if mode == "peak":
        return float(during.max() - baseline)
    tail = v[t >= end - GRADE_WINDOW_S]
    return float(tail.mean() - baseline)


@launch_testing.post_shutdown_test()
class TestDirections(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.truth = RECORDED.get("truth", [])
        cls.pulses = RECORDED["pulses"]
        write_samples_csv("thruster_directions.csv", truth=cls.truth)
        os.makedirs(OUT_DIR, exist_ok=True)
        with open(os.path.join(OUT_DIR, "thruster_directions_pulses.csv"), "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["axis", "pulse_start", "pulse_end", "settled_first"])
            for name, (start, end) in cls.pulses.items():
                writer.writerow([name, start, end, RECORDED["settled"].get(name)])

    def check_axis(self, name):
        _, _, field, min_change, units, mode, expected = AXES[name]
        self.assertIn(name, self.pulses, f"{name}: the sequence never got to this axis")
        delta = response(self.truth, field, *self.pulses[name], mode)
        self.assertIsNotNone(delta, f"{name}: no ground truth during its pulse")
        note = "" if RECORDED["settled"].get(name) else \
            " (note: the sub had not fully settled before this pulse)"
        self.assertGreater(delta, min_change,
                           f"{name}: {field} {'peaked at' if mode == 'peak' else 'changed'} "
                           f"{delta:+.3f} {units}, expected {expected} "
                           f"(at least +{min_change} {units}){note}")

    def test_Fx(self):
        self.check_axis("Fx")

    def test_Fy(self):
        self.check_axis("Fy")

    def test_Fz(self):
        self.check_axis("Fz")

    def test_Mx(self):
        self.check_axis("Mx")

    def test_My(self):
        self.check_axis("My")

    def test_Mz(self):
        self.check_axis("Mz")

    def test_dive_goes_down(self):
        self.assertIn("dive", RECORDED, "the sequence never got to the dive")
        drop = response(self.truth, "z", *RECORDED["dive"], "end")
        self.assertIsNotNone(drop, "no ground truth during the dive")
        self.assertLess(drop, -DIVE_MIN_DROP_M,
                        f"-Fz: z changed {drop:+.3f} m, expected the sub to go down "
                        f"(at least {DIVE_MIN_DROP_M} m)")
