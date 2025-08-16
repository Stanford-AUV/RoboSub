#!/usr/bin/env python3
"""
odometry_logger.py

ROS2 node that subscribes to nav_msgs/Odometry, logs:
- position: x, y, z
- orientation: roll, pitch, yaw (rad)

On shutdown, saves:
- CSV with all samples
- PNG plot (no interactive window)

Params:
  odom_topic (string): topic to subscribe (default: /odometry/filtered)
  out_prefix (string): output file prefix (default: odom_log)
  decimate   (int)   : keep 1 sample every N messages (default: 1 = keep all)
"""

import csv
import signal
from collections import deque
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import matplotlib
matplotlib.use("Agg")  # ensure no window is opened
import matplotlib.pyplot as plt

from msgs.msg import Float32Stamped

import numpy as np
import socket, struct  # ADD

class OdometryLogger(Node):
    def __init__(self):
        super().__init__("odometry_logger")

        # --- parameters ---
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("out_prefix", "odom_log")
        self.declare_parameter("decimate", 1)  # keep 1 in N messages

        # self.depth_topic = "depth"
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.out_prefix = self.get_parameter("out_prefix").get_parameter_value().string_value
        self.decimate = max(1, int(self.get_parameter("decimate").get_parameter_value().integer_value))

        # --- storage ---
        # Use deques in case you want to cap memory later; currently unlimited.
        self.t = deque()
        self.x = deque(); self.y = deque(); self.z = deque()
        self.roll = deque(); self.pitch = deque(); self.yaw = deque()
        self._count = 0
        self._t0 = None

        # --- subscriber ---
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.deepness = 0.0
        self.sub = self.create_subscription(Odometry, self.odom_topic, self._cb, qos)
        # self.depth = self.create_subscription(Float32Stamped, self.depth_topic, self._d_cb, qos)

        # --- UDP live sender (t, x, y) ---
        self.declare_parameter("udp_enable", True)
        self.declare_parameter("udp_host", "10.13.37.1")   # receiver IP/host
        self.declare_parameter("udp_port", 9999)          # receiver port
        self.declare_parameter("udp_rate_hz", 20.0)       # send rate (decimation)

        self.udp_enabled = bool(self.get_parameter("udp_enable").get_parameter_value().bool_value)
        self.udp_host = self.get_parameter("udp_host").get_parameter_value().string_value
        self.udp_port = int(self.get_parameter("udp_port").get_parameter_value().integer_value)
        self.udp_rate_hz = float(self.get_parameter("udp_rate_hz").get_parameter_value().double_value)

        self._udp_sock = None
        self._udp_dst = (self.udp_host, self.udp_port)
        self._udp_last_sent_ns = 0
        self._udp_period_ns = int(1e9 / max(self.udp_rate_hz, 1e-6))
        self._pack_fmt = "!fffff"  # network byte order: t_rel_s, x, y (float32)

        if self.udp_enabled:
            try:
                self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.get_logger().info(f"UDP sender ON -> {self.udp_host}:{self.udp_port} @ {self.udp_rate_hz:.1f} Hz")
            except Exception as e:
                self.get_logger().error(f"UDP socket init failed: {e}")
                self.udp_enabled = False


        self.get_logger().info(f"Listening on '{self.odom_topic}'. Press Ctrl+C to stop and save.")


    def remove_outliers(self):
        """Remove samples with any component beyond a z-score threshold."""
        if not self.t:
            return

        # Convert to numpy arrays for easier math
        arrs = {
            "t": np.array(self.t),
            "x": np.array(self.x),
            "y": np.array(self.y),
            "z": np.array(self.z),
            "roll": np.array(self.roll),
            "pitch": np.array(self.pitch),
            "yaw": np.array(self.yaw),
        }

        # Stack components except time
        data = np.column_stack([arrs["x"], arrs["y"], arrs["z"],
                                arrs["roll"], arrs["pitch"], arrs["yaw"]])

        # Z-score across each column
        z_scores = np.abs((data - np.nanmean(data, axis=0)) / np.nanstd(data, axis=0))

        # Mark a row as outlier if any component's z-score > threshold
        threshold = 4.0  # higher = less strict
        mask = (z_scores < threshold).all(axis=1)

        # Apply mask to all fields
        for key in arrs:
            arrs[key] = arrs[key][mask]

        # Replace deques with filtered lists
        self.t = list(arrs["t"])
        self.x = list(arrs["x"])
        self.y = list(arrs["y"])
        self.z = list(arrs["z"])
        self.roll = list(arrs["roll"])
        self.pitch = list(arrs["pitch"])
        self.yaw = list(arrs["yaw"])
        self.get_logger().info(f"Removed {np.count_nonzero(~mask)} outlier samples")

    # def _d_cb(self, msg: Float32Stamped):
    #     self.deepness = msg.data

    def _cb(self, msg: Odometry):
        self._count += 1
        if self._count % self.decimate != 0:
            return

        # time (relative seconds)
        tsec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = tsec
        t_rel = tsec - self._t0

        # position
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        # pz = self.deepness

        # orientation -> roll, pitch, yaw (radians)
        q = msg.pose.pose.orientation
        r, p, y = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=False)

        # store
        self.t.append(t_rel)
        self.x.append(px); self.y.append(py); self.z.append(pz)
        self.roll.append(r); self.pitch.append(p); self.yaw.append(y)

        # --- UDP send (t, x, y) ---
        if self.udp_enabled and self._udp_sock is not None:
            # Use message timestamp to pace sends
            now_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if now_ns - self._udp_last_sent_ns >= self._udp_period_ns:
                self._udp_last_sent_ns = now_ns
                try:
                    payload = struct.pack(self._pack_fmt, float(t_rel), float(px), float(py), float(pz), float(y))
                    self._udp_sock.sendto(payload, self._udp_dst)
                    self.get_logger().info(f"UDP sent packet")
                except Exception as e:
                    self.get_logger().warn(f"UDP send failed: {e}")


    # ---------- save helpers ----------
    def save_csv(self, path_csv: str):
        with open(path_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_m", "y_m", "z_m", "roll_rad", "pitch_rad", "yaw_rad"])
            for i in range(len(self.t)):
                w.writerow([self.t[i], self.x[i], self.y[i], self.z[i],
                            self.roll[i], self.pitch[i], self.yaw[i]])
        self.get_logger().info(f"Saved CSV: {path_csv}")

    def save_plot(self, path_png: str):
        if len(self.t) == 0:
            self.get_logger().warn("No samples recorded; skipping plot.")
            return

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        ax1, ax2 = axes

        ax1.plot(self.t, self.x, label="x")
        ax1.plot(self.t, self.y, label="y")
        ax1.plot(self.t, self.z, label="z")
        ax1.set_ylabel("Position (m)")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc="best")

        ax2.plot(self.t, self.roll, label="roll")
        ax2.plot(self.t, self.pitch, label="pitch")
        ax2.plot(self.t, self.yaw, label="yaw")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Orientation (rad)")
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc="best")

        fig.suptitle("Odometry (position & orientation)")
        fig.tight_layout()
        plt.savefig(path_png, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"Saved plot: {path_png}")

    def save_projections(self, path_png: str, stride: int = 1):
        """
        Save 2D trajectory projections: XY (top-down), XZ, YZ.
        `stride` lets you decimate points for dense logs.
        """
        if len(self.t) == 0:
            self.get_logger().warn("No samples recorded; skipping projections.")
            return

        import numpy as np
        import matplotlib.pyplot as plt

        xs = np.asarray(self.x)[::stride]
        ys = np.asarray(self.y)[::stride]
        zs = np.asarray(self.z)[::stride]

        fig, axes = plt.subplots(1, 3, figsize=(12, 4))
        ax_xy, ax_xz, ax_yz = axes

        # XY (top-down)
        ax_xy.plot(xs, ys, linewidth=1.5)
        ax_xy.plot(xs[:1], ys[:1], 'o', label='start')
        ax_xy.plot(xs[-1:], ys[-1:], 's', label='end')
        ax_xy.set_xlabel("X [m]"); ax_xy.set_ylabel("Y [m]")
        ax_xy.set_title("Top-down (XY)")
        ax_xy.grid(True, alpha=0.3)
        ax_xy.legend(loc="best")
        ax_xy.set_aspect('equal', adjustable='datalim')

        # XZ
        ax_xz.plot(xs, zs, linewidth=1.5)
        ax_xz.plot(xs[:1], zs[:1], 'o', label='start')
        ax_xz.plot(xs[-1:], zs[-1:], 's', label='end')
        ax_xz.set_xlabel("X [m]"); ax_xz.set_ylabel("Z [m]")
        ax_xz.set_title("Side (XZ)")
        ax_xz.grid(True, alpha=0.3)
        ax_xz.legend(loc="best")
        ax_xz.set_aspect('equal', adjustable='datalim')

        # YZ
        ax_yz.plot(ys, zs, linewidth=1.5)
        ax_yz.plot(ys[:1], zs[:1], 'o', label='start')
        ax_yz.plot(ys[-1:], zs[-1:], 's', label='end')
        ax_yz.set_xlabel("Y [m]"); ax_yz.set_ylabel("Z [m]")
        ax_yz.set_title("Side (YZ)")
        ax_yz.grid(True, alpha=0.3)
        ax_yz.legend(loc="best")
        ax_yz.set_aspect('equal', adjustable='datalim')

        fig.suptitle("Odometry Projections")
        fig.tight_layout()
        plt.savefig(path_png, dpi=150)
        plt.close(fig)
        self.get_logger().info(f"Saved projections: {path_png}")

def main():
    rclpy.init()
    node = OdometryLogger()

    # graceful shutdown: Ctrl+C → save files
    shutting_down = {"flag": False}

    def _handle_sigint(signum, frame):
        if shutting_down["flag"]:
            return
        shutting_down["flag"] = True
        node.get_logger().info("SIGINT received, saving logs...")

        node.remove_outliers()

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        base = f"{node.out_prefix}_{ts}"
        node.save_csv(base + ".csv")
        node.save_plot(base + ".png")
        node.save_projections(base + "_projections.png")
        
        # close UDP socket if open
        try:
            if getattr(node, "_udp_sock", None) is not None:
                node._udp_sock.close()
        except Exception:
            pass

        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    signal.signal(signal.SIGINT, _handle_sigint)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        _handle_sigint(None, None)


if __name__ == "__main__":
    main()
