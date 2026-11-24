"""Shared pieces for the sim tests in this folder (no test_ prefix, so it is not a test itself).

Test files import it after putting this folder on the import path:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from sim_helpers import ...
(launch_test loads a test file by path and does not add its folder to sys.path.)
"""
import csv
import os
import threading

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIM_STACK = os.path.join(REPO, "src", "main", "launch", "sim_stack.py")
# Recordings for plotting a failed run. Ignored by git; CI uploads the folder.
OUT_DIR = os.path.join(REPO, "tests", "sim", "out")

SAMPLE_FIELDS = ["t", "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz", "wx", "wy", "wz"]


def odom_sample(msg):
    """One nav_msgs/Odometry message as a dict of SAMPLE_FIELDS.

    t: the message's own stamp, which is sim time on both /odometry topics.
    roll, pitch, yaw: degrees, ROS signs (roll + = port side up, pitch + = nose down,
        yaw + = turned left), the same convention as scratch/watch_truth.py.
    vx, vy, vz (m/s) and wx, wy, wz (rad/s): msg.twist, in the sub's own frame (x forward,
        y left, z up). ground_truth.py rotates its velocity into the body frame and computes
        its rotation rate from consecutive orientations as q_prev* x q_now, which is also
        body frame; the EKF publishes its twist in base_link.
    """
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    roll, pitch, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    return {"t": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "x": p.x, "y": p.y, "z": p.z, "roll": roll, "pitch": pitch, "yaw": yaw,
            "vx": v.x, "vy": v.y, "vz": v.z, "wx": w.x, "wy": w.y, "wz": w.z}


class Recorder(Node):
    """Stores what the stack publishes, for grading. Runs on sim time.

    use_sim_time makes get_clock() follow /clock, so a test can time its own actions in
    sim seconds; the stored samples use each message's stamp either way.
    """

    def __init__(self, name="sim_test_recorder"):
        super().__init__(name, parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.truth = []
        self.ekf = []
        self.wrench_count = 0
        self.create_subscription(Odometry, "/odometry/ground_truth",
                                 lambda m: self.truth.append(odom_sample(m)), 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 lambda m: self.ekf.append(odom_sample(m)), 10)
        self.create_subscription(WrenchStamped, "/wrench", self._on_wrench, 10)

    def _on_wrench(self, _msg):
        self.wrench_count += 1

    def sim_now(self):
        """Current sim time in seconds (0.0 until the first /clock message arrives)."""
        return self.get_clock().now().nanoseconds * 1e-9


class Spinner:
    """Spins a node on a background thread, so its callbacks keep running while the
    test's main thread waits or sends commands."""

    def __init__(self, node):
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()

    def stop(self):
        self._executor.shutdown()
        self._thread.join(timeout=5)


def write_samples_csv(filename, **sources):
    """Write each list of samples (dicts from odom_sample) to tests/sim/out/<filename>,
    one row per sample, tagged with its source name (for example truth=..., ekf=...)."""
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, filename)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["source"] + SAMPLE_FIELDS)
        writer.writeheader()
        for source, samples in sources.items():
            for sample in samples:
                writer.writerow({"source": source, **sample})
    return path
