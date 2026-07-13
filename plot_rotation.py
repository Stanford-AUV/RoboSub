#!/usr/bin/env python3
"""Live time-series of the EKF-filtered rotation vs the raw IMU orientation.

Subscribes:
    /odometry/filtered  (nav_msgs/Odometry)   EKF output (filtered rotation)
    /imu/orientation    (sensor_msgs/Imu)     Xsens quaternion fed INTO the EKF

ROS spins in a background thread and only appends to deques; matplotlib runs
in the main thread on its own clock. This avoids the sensors_plot failure mode
where a blocking GUI + queue-10 dropped ~99% of messages.
"""

import argparse
import collections
import threading
import time

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu

# Okabe-Ito (CVD-safe): roll=blue, pitch=orange, yaw=green
COLORS = {"roll": "#0072B2", "pitch": "#E69F00", "yaw": "#009E73"}
AXES = ["roll", "pitch", "yaw"]


class Buffer:
    """Thread-safe (t, roll, pitch, yaw) ring buffer in degrees."""

    def __init__(self, maxlen=20000):
        self.lock = threading.Lock()
        self.t = collections.deque(maxlen=maxlen)
        self.rpy = {a: collections.deque(maxlen=maxlen) for a in AXES}

    def append(self, t, quat_xyzw):
        q = np.asarray(quat_xyzw, dtype=float)
        if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-6:
            return
        # extrinsic xyz == roll/pitch/yaw about fixed axes
        r, p, y = R.from_quat(q / np.linalg.norm(q)).as_euler("xyz", degrees=True)
        with self.lock:
            self.t.append(t)
            self.rpy["roll"].append(r)
            self.rpy["pitch"].append(p)
            self.rpy["yaw"].append(y)

    def snapshot(self):
        with self.lock:
            t = np.array(self.t)
            vals = {a: np.array(self.rpy[a]) for a in AXES}
        return t, vals


class RotationTap(Node):
    def __init__(self, ekf_buf, imu_buf):
        super().__init__("rotation_plot_tap")
        self.t0 = time.monotonic()
        self.ekf_buf = ekf_buf
        self.imu_buf = imu_buf
        self.n_ekf = 0
        self.n_imu = 0
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 50)
        self.create_subscription(Imu, "/imu/orientation", self._on_imu, 50)

    def _now(self):
        return time.monotonic() - self.t0

    def _on_odom(self, msg):
        o = msg.pose.pose.orientation
        self.ekf_buf.append(self._now(), [o.x, o.y, o.z, o.w])
        self.n_ekf += 1

    def _on_imu(self, msg):
        o = msg.orientation
        self.imu_buf.append(self._now(), [o.x, o.y, o.z, o.w])
        self.n_imu += 1


def unwrap_deg(vals):
    """Unwrap so a +/-180 crossing doesn't paint a vertical line."""
    return np.degrees(np.unwrap(np.radians(vals))) if len(vals) else vals


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--window", type=float, default=120.0,
                    help="seconds of history shown (default 120)")
    ap.add_argument("--redraw", type=float, default=0.25,
                    help="GUI redraw period in seconds")
    args = ap.parse_args()

    rclpy.init()
    ekf_buf, imu_buf = Buffer(), Buffer()
    node = RotationTap(ekf_buf, imu_buf)
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    plt.rcParams["axes.grid"] = True
    plt.rcParams["grid.alpha"] = 0.25
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    fig.canvas.manager.set_window_title("EKF vs IMU rotation")
    lines = {}
    for ax, name in zip(axs, AXES):
        c = COLORS[name]
        (lines[(name, "ekf")],) = ax.plot([], [], color=c, lw=2,
                                          label="EKF /odometry/filtered")
        (lines[(name, "imu")],) = ax.plot([], [], color="#666666", lw=1.2,
                                          ls="--", label="IMU /imu/orientation")
        ax.set_ylabel(f"{name} [deg]")
        ax.legend(loc="upper left", fontsize=8)
    axs[-1].set_xlabel("time [s]")

    try:
        while plt.fignum_exists(fig.number):
            te, ve = ekf_buf.snapshot()
            ti, vi = imu_buf.snapshot()
            now = time.monotonic() - node.t0
            t_lo = max(0.0, now - args.window)
            for ax, name in zip(axs, AXES):
                if len(te):
                    m = te >= t_lo
                    lines[(name, "ekf")].set_data(te[m], unwrap_deg(ve[name])[m])
                if len(ti):
                    m = ti >= t_lo
                    lines[(name, "imu")].set_data(ti[m], unwrap_deg(vi[name])[m])
                ax.relim()
                ax.autoscale_view()
                ax.set_xlim(t_lo, max(now, t_lo + 5.0))
            latest = (f"roll {ve['roll'][-1]:+7.2f}  pitch {ve['pitch'][-1]:+7.2f}  "
                      f"yaw {ve['yaw'][-1]:+7.2f} deg" if len(te) else "waiting for EKF...")
            fig.suptitle(f"EKF: {node.n_ekf} msgs | IMU: {node.n_imu} msgs | {latest}",
                         fontsize=10)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt.pause(args.redraw)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
