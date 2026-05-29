"""
Real-time PID visualizer.

Subscribes to /odometry/filtered, /desired/pose, /wrench.
Plots current vs desired position, error magnitude, and wrench output.
ROS2 spins in a background thread; matplotlib runs on the main thread.
"""

import threading
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import WrenchStamped

WINDOW_SECONDS = 30
MAX_POINTS = WINDOW_SECONDS * 60  # 60 Hz worst case


class PIDVisualizer(Node):
    def __init__(self):
        super().__init__("pid_visualizer")
        self.lock = threading.Lock()

        self.t0 = time.monotonic()
        self._make_buffers()

        self.create_subscription(Odometry, "/odometry/filtered", self._curr_cb, 10)
        self.create_subscription(Odometry, "/desired/pose", self._des_cb, 10)
        self.create_subscription(WrenchStamped, "/wrench", self._wrench_cb, 10)

        self.get_logger().info("PIDVisualizer listening — plot window opening.")

    def _make_buffers(self):
        n = MAX_POINTS
        self.t_curr = deque(maxlen=n)
        self.curr_x = deque(maxlen=n)
        self.curr_y = deque(maxlen=n)
        self.curr_z = deque(maxlen=n)

        self.t_des = deque(maxlen=n)
        self.des_x = deque(maxlen=n)
        self.des_y = deque(maxlen=n)
        self.des_z = deque(maxlen=n)

        self.t_err = deque(maxlen=n)
        self.err_mag = deque(maxlen=n)

        self.t_wrench = deque(maxlen=n)
        self.fx = deque(maxlen=n)
        self.fy = deque(maxlen=n)
        self.fz = deque(maxlen=n)
        self.tx = deque(maxlen=n)
        self.ty = deque(maxlen=n)
        self.tz = deque(maxlen=n)

        # last known values for cross-topic error computation
        self._last_curr = None
        self._last_des = None

    def _now(self):
        return time.monotonic() - self.t0

    def _curr_cb(self, msg: Odometry):
        t = self._now()
        pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        with self.lock:
            self.t_curr.append(t)
            self.curr_x.append(pos[0])
            self.curr_y.append(pos[1])
            self.curr_z.append(pos[2])
            self._last_curr = pos
            self._update_error(t)

    def _des_cb(self, msg: Odometry):
        t = self._now()
        pos = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ]
        )
        with self.lock:
            self.t_des.append(t)
            self.des_x.append(pos[0])
            self.des_y.append(pos[1])
            self.des_z.append(pos[2])
            self._last_des = pos
            self._update_error(t)

    def _update_error(self, t):
        if self._last_curr is not None and self._last_des is not None:
            err = np.linalg.norm(self._last_des - self._last_curr)
            self.t_err.append(t)
            self.err_mag.append(err)

    def _wrench_cb(self, msg: WrenchStamped):
        t = self._now()
        with self.lock:
            self.t_wrench.append(t)
            self.fx.append(msg.wrench.force.x)
            self.fy.append(msg.wrench.force.y)
            self.fz.append(msg.wrench.force.z)
            self.tx.append(msg.wrench.torque.x)
            self.ty.append(msg.wrench.torque.y)
            self.tz.append(msg.wrench.torque.z)


def _snap(buf):
    return list(buf)


def build_figure():
    fig, axes = plt.subplots(2, 3, figsize=(14, 7))
    fig.suptitle("PID Control Monitor", fontsize=13)
    fig.tight_layout(pad=2.5)

    ax_x, ax_y, ax_z = axes[0]
    ax_err, ax_force, ax_torque = axes[1]

    for ax, lbl in [(ax_x, "X (m)"), (ax_y, "Y (m)"), (ax_z, "Z (m)")]:
        ax.set_ylabel(lbl)
        ax.set_xlabel("t (s)")
        ax.grid(True, alpha=0.3)

    ax_err.set_ylabel("‖error‖ (m)")
    ax_err.set_xlabel("t (s)")
    ax_err.set_title("Position Error Magnitude")
    ax_err.grid(True, alpha=0.3)

    ax_force.set_ylabel("Force (N)")
    ax_force.set_xlabel("t (s)")
    ax_force.set_title("Wrench — Force")
    ax_force.grid(True, alpha=0.3)

    ax_torque.set_ylabel("Torque (N·m)")
    ax_torque.set_xlabel("t (s)")
    ax_torque.set_title("Wrench — Torque")
    ax_torque.grid(True, alpha=0.3)

    return fig, axes


def main(args=None):
    rclpy.init(args=args)
    node = PIDVisualizer()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fig, axes = build_figure()
    ax_x, ax_y, ax_z = axes[0]
    ax_err, ax_force, ax_torque = axes[1]

    def update(_frame):
        with node.lock:
            tc = _snap(node.t_curr)
            cx = _snap(node.curr_x)
            cy = _snap(node.curr_y)
            cz = _snap(node.curr_z)
            td = _snap(node.t_des)
            dx = _snap(node.des_x)
            dy = _snap(node.des_y)
            dz = _snap(node.des_z)
            te = _snap(node.t_err)
            em = _snap(node.err_mag)
            tw = _snap(node.t_wrench)
            fx = _snap(node.fx)
            fy = _snap(node.fy)
            fz = _snap(node.fz)
            tx = _snap(node.tx)
            ty = _snap(node.ty)
            tz = _snap(node.tz)

        for ax in axes.flat:
            ax.cla()
            ax.grid(True, alpha=0.3)

        # Position X
        ax_x.set_title("Position X")
        ax_x.set_ylabel("X (m)")
        if tc:
            ax_x.plot(tc, cx, color="tab:blue", label="current", lw=1.5)
        if td:
            ax_x.plot(
                td, dx, color="tab:orange", linestyle="--", label="desired", lw=1.2
            )
        ax_x.legend(fontsize=7)

        # Position Y
        ax_y.set_title("Position Y")
        ax_y.set_ylabel("Y (m)")
        if tc:
            ax_y.plot(tc, cy, color="tab:blue", lw=1.5)
        if td:
            ax_y.plot(td, dy, color="tab:orange", linestyle="--", lw=1.2)

        # Position Z
        ax_z.set_title("Position Z")
        ax_z.set_ylabel("Z (m)")
        if tc:
            ax_z.plot(tc, cz, color="tab:blue", lw=1.5)
        if td:
            ax_z.plot(td, dz, color="tab:orange", linestyle="--", lw=1.2)

        # Error magnitude
        ax_err.set_title("Position Error Magnitude")
        ax_err.set_ylabel("‖error‖ (m)")
        if te:
            ax_err.plot(te, em, color="tab:red", lw=1.5)
            ax_err.axhline(0, color="gray", lw=0.8)

        # Force
        ax_force.set_title("Wrench — Force")
        ax_force.set_ylabel("Force (N)")
        if tw:
            ax_force.plot(tw, fx, label="fx", lw=1.2)
            ax_force.plot(tw, fy, label="fy", lw=1.2)
            ax_force.plot(tw, fz, label="fz", lw=1.2)
            ax_force.axhline(0, color="gray", lw=0.8)
            ax_force.legend(fontsize=7)

        # Torque
        ax_torque.set_title("Wrench — Torque")
        ax_torque.set_ylabel("Torque (N·m)")
        if tw:
            ax_torque.plot(tw, tx, label="tx", lw=1.2)
            ax_torque.plot(tw, ty, label="ty", lw=1.2)
            ax_torque.plot(tw, tz, label="tz", lw=1.2)
            ax_torque.axhline(0, color="gray", lw=0.8)
            ax_torque.legend(fontsize=7)

        for ax in axes.flat:
            ax.set_xlabel("t (s)")

        fig.tight_layout(pad=2.0)

    ani = animation.FuncAnimation(fig, update, interval=200, cache_frame_data=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
