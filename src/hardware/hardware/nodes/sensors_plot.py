import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from msgs.msg import PingerStamped, SensorsStamped
from hardware.pinger import detector as pinger_detector

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
import matplotlib.pyplot as plt
import collections
import math
import os
import threading
from types import SimpleNamespace

import numpy as np

# Per-series storage cap: bounds memory AND keeps the snapshot copy cheap.
# ~8 min of 400 Hz data; slower topics keep the whole run.
HISTORY_MAX = 200_000
# Max points actually drawn per line: keeps a redraw O(1) no matter how
# long the run is (drawing 200k-point lines x 30 series took seconds and
# was the main source of plot lag late in a run).
PLOT_POINTS = 2000


def _deque():
    return collections.deque(maxlen=HISTORY_MAX)


def _decimate(seq):
    """Bounded copy of a series for drawing (every k-th point)."""
    n = len(seq)
    if n <= PLOT_POINTS:
        return list(seq)
    step = -(-n // PLOT_POINTS)  # ceil division
    return list(seq)[::step]


class SensorsPlot(Node):
    """Plot the pieces of the localization puzzle: what we FEED the EKF (IMU
    absolute rotation, DVL velocity) next to what the EKF PUTS OUT (rotation,
    velocity, position, acceleration).

    Drawing/savefig runs in a BACKGROUND thread: the executor thread only
    ever appends samples, so subscriptions drain continuously and the PNG
    shows fresh data (drawing inline used to block the executor 1-2 s per
    frame; messages then aged in the deep queues and appeared ~a minute
    late). All panels share one x-scale (0 .. newest stamp).
    """

    # Deep queues as a safety net for scheduling hiccups; with the
    # background draw the executor drains these continuously, so they no
    # longer add latency in steady state.
    QUEUE_DEPTH = 1000

    def __init__(self):
        super().__init__("sensors_plot")

        # ---- EKF INPUTS ----
        # Absolute orientation measurement we feed the EKF (imu.py output).
        self.imu_in_sub = self.create_subscription(
            Imu, "/imu/orientation", self.imu_in_callback, self.QUEUE_DEPTH
        )
        # DVL body velocity.
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/velocity", self.velocity_callback,
            self.QUEUE_DEPTH,
        )

        # ---- EKF OUTPUTS ----
        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/filtered", self.odom_callback, self.QUEUE_DEPTH
        )
        self.ekf_accel_sub = self.create_subscription(
            AccelWithCovarianceStamped, "/accel/filtered", self.ekf_accel_callback,
            self.QUEUE_DEPTH,
        )
        # Desired position setpoint (planning -> control).
        self.desired_sub = self.create_subscription(
            Odometry, "/desired/pose", self.desired_callback, self.QUEUE_DEPTH
        )
        # Raw hardware sensor bundle (temperatures, current, voltage) - same
        # source the depth sensor node reads.
        self.arduino_sub = self.create_subscription(
            SensorsStamped, "/arduino/sensors", self.arduino_callback,
            self.QUEUE_DEPTH,
        )

        self.pinger_sub = self.create_subscription(
            PingerStamped, "/pinger/levels", self.pinger_callback,
            self.QUEUE_DEPTH,
        )

        self.fig = plt.figure(figsize=(24, 12))

        # Row 1: INPUTS + EKF rotation + temperatures.
        # Row 2: EKF outputs + pinger.  Row 3: power electronics + desired.
        self.ax_imu_in = self.fig.add_subplot(341)
        self.ax_dvl_vel = self.fig.add_subplot(342)
        self.ax_ekf_rot = self.fig.add_subplot(343)
        self.ax_temp = self.fig.add_subplot(344)
        self.ax_ekf_vel = self.fig.add_subplot(345)
        self.ax_ekf_pos = self.fig.add_subplot(346)
        self.ax_ekf_accel = self.fig.add_subplot(347)
        # Pinger: 4 hydrophone levels + front/back decision (power was
        # dropped -- it is just the neighboring current x voltage panels).
        self.ax_pinger = self.fig.add_subplot(348)
        self.ax_current = self.fig.add_subplot(349)
        self.ax_voltage = self.fig.add_subplot(3, 4, 10)
        self.ax_des_pos = self.fig.add_subplot(3, 4, 11)
        self.ax_des_rot = self.fig.add_subplot(3, 4, 12)

        # Series storage: appended by subscription callbacks (executor
        # thread) under _lock; snapshotted by the draw request.
        self._lock = threading.Lock()

        self.imu_in_time = _deque()
        self.imu_in_roll_history = _deque()
        self.imu_in_pitch_history = _deque()
        self.imu_in_yaw_history = _deque()

        self.vel_time = _deque()
        self.vel_x_history = _deque()
        self.vel_y_history = _deque()
        self.vel_z_history = _deque()

        self.rot_time = _deque()
        self.rot_x_history = _deque()
        self.rot_y_history = _deque()
        self.rot_z_history = _deque()

        self.ekf_vel_time = _deque()
        self.ekf_vel_x_history = _deque()
        self.ekf_vel_y_history = _deque()
        self.ekf_vel_z_history = _deque()

        self.ekf_pos_time = _deque()
        self.ekf_pos_x_history = _deque()
        self.ekf_pos_y_history = _deque()
        self.ekf_pos_z_history = _deque()

        self.ekf_accel_time = _deque()
        self.ekf_accel_x_history = _deque()
        self.ekf_accel_y_history = _deque()
        self.ekf_accel_z_history = _deque()

        self.des_pos_time = _deque()
        self.des_pos_x_history = _deque()
        self.des_pos_y_history = _deque()
        self.des_pos_z_history = _deque()
        self.des_rot_roll_history = _deque()
        self.des_rot_pitch_history = _deque()
        self.des_rot_yaw_history = _deque()

        self.arduino_time = _deque()
        self.ext_temp_history = _deque()
        self.int_temp1_history = _deque()
        self.int_temp2_history = _deque()
        self.current_history = _deque()
        self.voltage_history = _deque()

        self.pinger_time = _deque()
        self.pinger_levels_history = [_deque(), _deque(), _deque(), _deque()]
        self.pinger_front_history = _deque()

        self._series_keys = [
            "imu_in_time", "imu_in_roll_history", "imu_in_pitch_history",
            "imu_in_yaw_history",
            "vel_time", "vel_x_history", "vel_y_history", "vel_z_history",
            "rot_time", "rot_x_history", "rot_y_history", "rot_z_history",
            "ekf_vel_time", "ekf_vel_x_history", "ekf_vel_y_history",
            "ekf_vel_z_history",
            "ekf_pos_time", "ekf_pos_x_history", "ekf_pos_y_history",
            "ekf_pos_z_history",
            "ekf_accel_time", "ekf_accel_x_history", "ekf_accel_y_history",
            "ekf_accel_z_history",
            "des_pos_time", "des_pos_x_history", "des_pos_y_history",
            "des_pos_z_history", "des_rot_roll_history",
            "des_rot_pitch_history", "des_rot_yaw_history",
            "arduino_time", "ext_temp_history", "int_temp1_history",
            "int_temp2_history", "current_history", "voltage_history",
            "pinger_time", "pinger_front_history",
        ]
        self._time_keys = [
            "imu_in_time", "vel_time", "rot_time", "ekf_vel_time",
            "ekf_pos_time", "ekf_accel_time", "des_pos_time",
            "arduino_time", "pinger_time",
        ]

        self._drawing = False
        self._t0 = None  # first header stamp seen; time axis is relative to it
        self.create_timer(2.0, self.request_draw)

    def _stamp(self, msg):
        """Seconds since start-of-run, from the message HEADER stamp (true
        sample time), not arrival time - arrival time smears samples that
        sat in the queue while the plotter was busy saving."""
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if t == 0.0:  # unstamped publisher: fall back to node clock
            t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def imu_in_callback(self, msg: Imu):
        q = msg.orientation
        r, p, y = self.quaternion_to_rpy(q.w, q.x, q.y, q.z)
        with self._lock:
            self.imu_in_time.append(self._stamp(msg))
            self.imu_in_roll_history.append(r)
            self.imu_in_pitch_history.append(p)
            self.imu_in_yaw_history.append(y)

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        with self._lock:
            self.vel_time.append(self._stamp(msg))
            self.vel_x_history.append(msg.twist.twist.linear.x)
            self.vel_y_history.append(msg.twist.twist.linear.y)
            self.vel_z_history.append(msg.twist.twist.linear.z)

    def odom_callback(self, msg: Odometry):
        quat = msg.pose.pose.orientation
        r, p, y = self.quaternion_to_rpy(quat.w, quat.x, quat.y, quat.z)
        pos = msg.pose.pose.position
        with self._lock:
            t = self._stamp(msg)
            self.rot_time.append(t)
            self.rot_x_history.append(r)
            self.rot_y_history.append(p)
            self.rot_z_history.append(y)

            self.ekf_vel_time.append(t)
            self.ekf_vel_x_history.append(msg.twist.twist.linear.x)
            self.ekf_vel_y_history.append(msg.twist.twist.linear.y)
            self.ekf_vel_z_history.append(msg.twist.twist.linear.z)

            self.ekf_pos_time.append(t)
            self.ekf_pos_x_history.append(pos.x)
            self.ekf_pos_y_history.append(pos.y)
            self.ekf_pos_z_history.append(pos.z)

    def ekf_accel_callback(self, msg: AccelWithCovarianceStamped):
        with self._lock:
            self.ekf_accel_time.append(self._stamp(msg))
            self.ekf_accel_x_history.append(msg.accel.accel.linear.x)
            self.ekf_accel_y_history.append(msg.accel.accel.linear.y)
            self.ekf_accel_z_history.append(msg.accel.accel.linear.z)

    def desired_callback(self, msg: Odometry):
        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        r, p, y = self.quaternion_to_rpy(q.w, q.x, q.y, q.z)
        with self._lock:
            self.des_pos_time.append(self._stamp(msg))
            self.des_pos_x_history.append(pos.x)
            self.des_pos_y_history.append(pos.y)
            self.des_pos_z_history.append(pos.z)
            self.des_rot_roll_history.append(r)
            self.des_rot_pitch_history.append(p)
            self.des_rot_yaw_history.append(y)

    def arduino_callback(self, msg: SensorsStamped):
        with self._lock:
            self.arduino_time.append(self._stamp(msg))
            self.ext_temp_history.append(msg.external_temperature)
            self.int_temp1_history.append(msg.internal_temperature1)
            self.int_temp2_history.append(msg.internal_temperature2)
            self.current_history.append(msg.current)
            self.voltage_history.append(msg.voltage)

    def pinger_callback(self, msg: PingerStamped):
        with self._lock:
            self.pinger_time.append(self._stamp(msg))
            for i in range(4):
                self.pinger_levels_history[i].append(msg.levels[i])
            self.pinger_front_history.append(1.0 if msg.front else 0.0)

    def quaternion_to_rpy(self, q_w, q_x, q_y, q_z):
        # Roll (x-axis)
        sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
        cosr_cosp = 1 - 2 * (q_x**2 + q_y**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis)
        pitch = 2 * (q_w * q_y - q_z * q_x)
        if abs(pitch) >= 1:
            pitch = math.copysign(math.pi / 2, pitch)
        else:
            pitch = math.asin(pitch)

        # Yaw (z-axis)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y**2 + q_z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def request_draw(self):
        """Timer callback: snapshot the data and hand it to a background
        draw thread. Never blocks the executor; skips a frame if the
        previous draw is still running."""
        if self._drawing:
            return
        with self._lock:
            snap = {k: _decimate(getattr(self, k)) for k in self._series_keys}
            snap["pinger_levels_history"] = [
                _decimate(d) for d in self.pinger_levels_history
            ]
        self._drawing = True
        threading.Thread(target=self._draw, args=(snap,), daemon=True).start()

    def _draw(self, snap):
        try:
            self._draw_inner(SimpleNamespace(**snap))
        except Exception as e:  # a draw glitch must never kill the node
            self.get_logger().error(f"plot draw failed: {e}")
        finally:
            self._drawing = False

    def _draw_inner(self, s):
        axes = (
            self.ax_imu_in,
            self.ax_dvl_vel,
            self.ax_ekf_rot,
            self.ax_ekf_vel,
            self.ax_ekf_pos,
            self.ax_ekf_accel,
            self.ax_des_pos,
            self.ax_temp,
            self.ax_current,
            self.ax_voltage,
            self.ax_pinger,
            self.ax_des_rot,
        )
        for ax in axes:
            ax.cla()

        # ---- INPUT: IMU absolute rotation fed to the EKF ----
        self.ax_imu_in.set_title("IMU Rotation IN (/imu/orientation)")
        self.ax_imu_in.plot(s.imu_in_time, np.unwrap(s.imu_in_roll_history), "r-", label="ROLL")
        self.ax_imu_in.plot(s.imu_in_time, np.unwrap(s.imu_in_pitch_history), "g-", label="PITCH")
        self.ax_imu_in.plot(s.imu_in_time, np.unwrap(s.imu_in_yaw_history), "b-", label="YAW")
        self.ax_imu_in.set_ylabel("Rotation (rad)")

        # ---- INPUT: DVL velocity ----
        self.ax_dvl_vel.set_title("DVL Velocity (/velocity)")
        self.ax_dvl_vel.plot(s.vel_time, s.vel_x_history, "r-", label="X")
        self.ax_dvl_vel.plot(s.vel_time, s.vel_y_history, "g-", label="Y")
        self.ax_dvl_vel.plot(s.vel_time, s.vel_z_history, "b-", label="Z")
        self.ax_dvl_vel.set_ylabel("Velocity (m/s)")

        # ---- OUTPUT: EKF rotation ----
        self.ax_ekf_rot.set_title("EKF Rotation (/odometry/filtered)")
        self.ax_ekf_rot.plot(s.rot_time, np.unwrap(s.rot_x_history), "r-", label="ROLL")
        self.ax_ekf_rot.plot(s.rot_time, np.unwrap(s.rot_y_history), "g-", label="PITCH")
        self.ax_ekf_rot.plot(s.rot_time, np.unwrap(s.rot_z_history), "b-", label="YAW")
        self.ax_ekf_rot.set_ylabel("Rotation (rad)")

        # ---- OUTPUT: EKF velocity ----
        self.ax_ekf_vel.set_title("EKF Velocity (/odometry/filtered)")
        self.ax_ekf_vel.plot(s.ekf_vel_time, s.ekf_vel_x_history, "r-", label="X")
        self.ax_ekf_vel.plot(s.ekf_vel_time, s.ekf_vel_y_history, "g-", label="Y")
        self.ax_ekf_vel.plot(s.ekf_vel_time, s.ekf_vel_z_history, "b-", label="Z")
        self.ax_ekf_vel.set_ylabel("Velocity (m/s)")

        # ---- OUTPUT: EKF position ----
        # Z is pinned by depth; X/Y dead-reckon from DVL velocity and drift
        # with no absolute horizontal fix.
        self.ax_ekf_pos.set_title("EKF Position (/odometry/filtered)")
        self.ax_ekf_pos.plot(s.ekf_pos_time, s.ekf_pos_x_history, "r-", label="X")
        self.ax_ekf_pos.plot(s.ekf_pos_time, s.ekf_pos_y_history, "g-", label="Y")
        self.ax_ekf_pos.plot(s.ekf_pos_time, s.ekf_pos_z_history, "b-", label="Z")
        self.ax_ekf_pos.set_ylabel("Position (m)")

        # ---- OUTPUT: EKF acceleration ----
        self.ax_ekf_accel.set_title("EKF Acceleration (/accel/filtered)")
        self.ax_ekf_accel.plot(s.ekf_accel_time, s.ekf_accel_x_history, "r-", label="X")
        self.ax_ekf_accel.plot(s.ekf_accel_time, s.ekf_accel_y_history, "g-", label="Y")
        self.ax_ekf_accel.plot(s.ekf_accel_time, s.ekf_accel_z_history, "b-", label="Z")
        self.ax_ekf_accel.set_ylabel("Acceleration (m/s²)")

        # ---- DESIRED: position setpoint (/desired/pose) ----
        self.ax_des_pos.set_title("Desired Position (/desired/pose)")
        self.ax_des_pos.plot(s.des_pos_time, s.des_pos_x_history, "r-", label="X")
        self.ax_des_pos.plot(s.des_pos_time, s.des_pos_y_history, "g-", label="Y")
        self.ax_des_pos.plot(s.des_pos_time, s.des_pos_z_history, "b-", label="Z")
        self.ax_des_pos.set_ylabel("Position (m)")

        # ---- DESIRED: orientation setpoint (/desired/pose) ----
        self.ax_des_rot.set_title("Desired Orientation (/desired/pose)")
        self.ax_des_rot.plot(s.des_pos_time, np.unwrap(s.des_rot_roll_history), "r-", label="ROLL")
        self.ax_des_rot.plot(s.des_pos_time, np.unwrap(s.des_rot_pitch_history), "g-", label="PITCH")
        self.ax_des_rot.plot(s.des_pos_time, np.unwrap(s.des_rot_yaw_history), "b-", label="YAW")
        self.ax_des_rot.set_ylabel("Rotation (rad)")

        # ---- HARDWARE: temperatures (/arduino/sensors) ----
        self.ax_temp.set_title("Temperature (/arduino/sensors)")
        self.ax_temp.plot(s.arduino_time, s.ext_temp_history, "b-", label="EXTERNAL")
        self.ax_temp.plot(s.arduino_time, s.int_temp1_history, "r-", label="INTERNAL 1")
        self.ax_temp.plot(s.arduino_time, s.int_temp2_history, "m-", label="INTERNAL 2")
        self.ax_temp.set_ylabel("Temperature (°C)")

        # ---- HARDWARE: battery current ----
        self.ax_current.set_title("Current (/arduino/sensors)")
        self.ax_current.plot(s.arduino_time, s.current_history, "r-", label="CURRENT")
        self.ax_current.set_ylabel("Current (A)")

        # ---- HARDWARE: battery voltage ----
        self.ax_voltage.set_title("Voltage (/arduino/sensors)")
        self.ax_voltage.plot(s.arduino_time, s.voltage_history, "g-", label="VOLTAGE")
        self.ax_voltage.set_ylabel("Voltage (V)")

        # ---- PINGER: hydrophone levels + front/back decision ----
        self.ax_pinger.set_title(
            f"Pinger levels @ {pinger_detector.targetFrequency:.0f} Hz")
        for ch, color in enumerate(("b", "r", "m", "c")):
            side = "front" if ch in pinger_detector.FRONT_CHANNELS else "back"
            self.ax_pinger.plot(
                s.pinger_time, s.pinger_levels_history[ch],
                color + "-", label=f"ch{ch} ({side})")
        self.ax_pinger.axhline(
            pinger_detector.baseThreshold, color="k", linestyle="--",
            linewidth=0.8, label="threshold")
        self.ax_pinger.set_ylabel("Normalized level")
        self.ax_pinger.set_ylim(-0.05, 1.05)
        # Latched decision as a big bold label in the bottom-left corner.
        if s.pinger_front_history:
            self.ax_pinger.text(
                0.02, 0.04,
                "FRONT" if s.pinger_front_history[-1] else "BACK",
                transform=self.ax_pinger.transAxes, fontsize=26,
                fontweight="bold", verticalalignment="bottom")

        # One shared time scale for every panel: 0 .. newest stamp seen.
        t_max = max(
            (seq[-1] for seq in (getattr(s, k) for k in self._time_keys)
             if seq),
            default=1.0,
        )
        for ax in axes:
            ax.set_xlim(0.0, max(t_max * 1.02, 1.0))
            ax.set_xlabel("Time (s)")
            ax.grid(True)
            legend_size = "x-small" if ax is self.ax_pinger else None
            ax.legend(fontsize=legend_size, loc="upper left")

        self.fig.tight_layout()
        # Land the PNG in the repo root, NOT the launch CWD. ros2 launch starts
        # nodes from $HOME, so the old cwd-relative path wrote to ~/sensors_plot.png
        # where nobody was looking. Prefer $ROBOSUB_DIR, then ~/RoboSub, then cwd.
        out_dir = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
        if not os.path.isdir(out_dir):
            out_dir = os.getcwd()
        out_path = os.path.join(out_dir, "sensors_plot.png")
        self.fig.savefig(out_path)
        if not getattr(self, "_logged_out_path", False):
            self.get_logger().info(f"writing sensors plot to {out_path}")
            self._logged_out_path = True


def main(args=None):
    rclpy.init(args=args)
    node = SensorsPlot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        plt.close("all")


if __name__ == "__main__":
    main()
