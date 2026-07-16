import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from msgs.msg import PingerStamped, SensorsStamped
from hardware.pinger import detector as pinger_detector

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
# Aggressive path simplification: rasterizing the line paths dominates the
# frame time on the Orin; visually lossless for time-series at this size.
matplotlib.rcParams["path.simplify"] = True
matplotlib.rcParams["path.simplify_threshold"] = 1.0
import matplotlib.pyplot as plt
import collections
import math
import os
import threading
from types import SimpleNamespace

import numpy as np

# The plot shows a sliding window of the last WINDOW_S seconds; samples
# older than that are pruned at snapshot time (they can never be shown
# again), so storage stays ~window-sized regardless of run length.
WINDOW_S = 20.0
# Backstop cap in case pruning falls behind (e.g. a stamp glitch).
HISTORY_MAX = 50_000
# Max points actually drawn per line: at dpi 70 a panel is ~560 px wide,
# so 600 points is still >= 1 point per pixel; fewer points = faster
# rasterization, which dominates the frame time.
PLOT_POINTS = 600


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

        # time key -> the series that share its timestamps (appended
        # together atomically, so lengths always match).
        self._groups = {
            "imu_in_time": ["imu_in_roll_history", "imu_in_pitch_history",
                            "imu_in_yaw_history"],
            "vel_time": ["vel_x_history", "vel_y_history", "vel_z_history"],
            "rot_time": ["rot_x_history", "rot_y_history", "rot_z_history"],
            "ekf_vel_time": ["ekf_vel_x_history", "ekf_vel_y_history",
                             "ekf_vel_z_history"],
            "ekf_pos_time": ["ekf_pos_x_history", "ekf_pos_y_history",
                             "ekf_pos_z_history"],
            "ekf_accel_time": ["ekf_accel_x_history", "ekf_accel_y_history",
                               "ekf_accel_z_history"],
            "des_pos_time": ["des_pos_x_history", "des_pos_y_history",
                             "des_pos_z_history", "des_rot_roll_history",
                             "des_rot_pitch_history", "des_rot_yaw_history"],
            "arduino_time": ["ext_temp_history", "int_temp1_history",
                             "int_temp2_history", "current_history",
                             "voltage_history"],
            "pinger_time": ["pinger_front_history"],
        }

        # ---- Persistent artists: titles/labels/legends/lines are created
        # ONCE here; each frame only calls set_data on the lines. Clearing
        # and re-plotting 12 panels (cla + plot + legend) cost ~1.5 s/frame
        # on the Orin; updating line data is ~10x cheaper.
        self._panels = []

        def panel(ax, title, ylabel, time_key, series):
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.set_xlabel("Time (s)")
            ax.grid(True)
            lines = [(ax.plot([], [], style, label=label)[0], key, unwrap)
                     for key, style, label, unwrap in series]
            ax.legend(loc="upper left", fontsize="x-small")
            self._panels.append((ax, time_key, lines))

        panel(self.ax_imu_in, "IMU Rotation IN (/imu/orientation)",
              "Rotation (rad)", "imu_in_time",
              [("imu_in_roll_history", "r-", "ROLL", True),
               ("imu_in_pitch_history", "g-", "PITCH", True),
               ("imu_in_yaw_history", "b-", "YAW", True)])
        panel(self.ax_dvl_vel, "DVL Velocity (/velocity)",
              "Velocity (m/s)", "vel_time",
              [("vel_x_history", "r-", "X", False),
               ("vel_y_history", "g-", "Y", False),
               ("vel_z_history", "b-", "Z", False)])
        panel(self.ax_ekf_rot, "EKF Rotation (/odometry/filtered)",
              "Rotation (rad)", "rot_time",
              [("rot_x_history", "r-", "ROLL", True),
               ("rot_y_history", "g-", "PITCH", True),
               ("rot_z_history", "b-", "YAW", True)])
        panel(self.ax_ekf_vel, "EKF Velocity (/odometry/filtered)",
              "Velocity (m/s)", "ekf_vel_time",
              [("ekf_vel_x_history", "r-", "X", False),
               ("ekf_vel_y_history", "g-", "Y", False),
               ("ekf_vel_z_history", "b-", "Z", False)])
        # Z is pinned by depth; X/Y dead-reckon from DVL velocity and drift
        # with no absolute horizontal fix.
        panel(self.ax_ekf_pos, "EKF Position (/odometry/filtered)",
              "Position (m)", "ekf_pos_time",
              [("ekf_pos_x_history", "r-", "X", False),
               ("ekf_pos_y_history", "g-", "Y", False),
               ("ekf_pos_z_history", "b-", "Z", False)])
        panel(self.ax_ekf_accel, "EKF Acceleration (/accel/filtered)",
              "Acceleration (m/s²)", "ekf_accel_time",
              [("ekf_accel_x_history", "r-", "X", False),
               ("ekf_accel_y_history", "g-", "Y", False),
               ("ekf_accel_z_history", "b-", "Z", False)])
        panel(self.ax_des_pos, "Desired Position (/desired/pose)",
              "Position (m)", "des_pos_time",
              [("des_pos_x_history", "r-", "X", False),
               ("des_pos_y_history", "g-", "Y", False),
               ("des_pos_z_history", "b-", "Z", False)])
        panel(self.ax_des_rot, "Desired Orientation (/desired/pose)",
              "Rotation (rad)", "des_pos_time",
              [("des_rot_roll_history", "r-", "ROLL", True),
               ("des_rot_pitch_history", "g-", "PITCH", True),
               ("des_rot_yaw_history", "b-", "YAW", True)])
        panel(self.ax_temp, "Temperature (/arduino/sensors)",
              "Temperature (°C)", "arduino_time",
              [("ext_temp_history", "b-", "EXTERNAL", False),
               ("int_temp1_history", "r-", "INTERNAL 1", False),
               ("int_temp2_history", "m-", "INTERNAL 2", False)])
        panel(self.ax_current, "Current (/arduino/sensors)",
              "Current (A)", "arduino_time",
              [("current_history", "r-", "CURRENT", False)])
        panel(self.ax_voltage, "Voltage (/arduino/sensors)",
              "Voltage (V)", "arduino_time",
              [("voltage_history", "g-", "VOLTAGE", False)])

        # Pinger panel: 4 level lines + threshold + big bold FRONT/BACK.
        self.ax_pinger.set_title(
            f"Pinger levels @ {pinger_detector.targetFrequency:.0f} Hz")
        self.ax_pinger.set_ylabel("Normalized level")
        self.ax_pinger.set_xlabel("Time (s)")
        self.ax_pinger.set_ylim(-0.05, 1.05)
        self.ax_pinger.grid(True)
        self._pinger_lines = []
        for ch, color in enumerate(("b", "r", "m", "c")):
            side = "front" if ch in pinger_detector.FRONT_CHANNELS else "back"
            self._pinger_lines.append(self.ax_pinger.plot(
                [], [], color + "-", label=f"ch{ch} ({side})")[0])
        self.ax_pinger.axhline(
            pinger_detector.baseThreshold, color="k", linestyle="--",
            linewidth=0.8, label="threshold")
        self.ax_pinger.legend(loc="upper left", fontsize="x-small")
        self._pinger_text = self.ax_pinger.text(
            0.02, 0.04, "", transform=self.ax_pinger.transAxes,
            fontsize=26, fontweight="bold", verticalalignment="bottom")

        self._drawing = False
        self._layout_done = False
        self._t0 = None  # first header stamp seen; time axis is relative to it
        # Fast tick: request_draw skips while a draw is in flight, so the
        # effective frame rate self-throttles to the draw duration.
        self.create_timer(0.5, self.request_draw)

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
            t_max = max(
                (getattr(self, tk)[-1] for tk in self._groups
                 if getattr(self, tk)),
                default=0.0,
            )
            t_left = t_max - WINDOW_S
            snap = {"t_max": t_max}
            for tk, series_keys in self._groups.items():
                times = getattr(self, tk)
                cols = [getattr(self, sk) for sk in series_keys]
                if tk == "pinger_time":
                    cols = cols + self.pinger_levels_history
                # Prune everything that scrolled out of the window: it can
                # never be shown again, so drop it from storage too.
                while times and times[0] < t_left:
                    times.popleft()
                    for c in cols:
                        c.popleft()
                snap[tk] = _decimate(times)
                for sk, c in zip(series_keys, cols):
                    snap[sk] = _decimate(c)
            snap["pinger_levels_history"] = [
                _decimate(d) for d in self.pinger_levels_history
            ]
        self._drawing = True
        threading.Thread(target=self._draw, args=(snap,), daemon=True).start()

    def _draw(self, snap):
        try:
            self._draw_inner(snap)
        except Exception as e:  # a draw glitch must never kill the node
            self.get_logger().error(f"plot draw failed: {e}")
        finally:
            self._drawing = False

    def _draw_inner(self, snap):
        # Update line data in place (artists are created once in __init__).
        for ax, time_key, lines in self._panels:
            t = snap[time_key]
            for ln, key, unwrap in lines:
                y = snap[key]
                ln.set_data(t, np.unwrap(y) if unwrap else y)
            if t:  # rescale y to the data now in the window
                ax.relim()
                ax.autoscale_view(scalex=False)

        t = snap["pinger_time"]
        for ln, y in zip(self._pinger_lines, snap["pinger_levels_history"]):
            ln.set_data(t, y)
        front = snap["pinger_front_history"]
        self._pinger_text.set_text(
            ("FRONT" if front[-1] else "BACK") if front else "")

        # One shared time scale for every panel: a sliding WINDOW_S-second
        # window ending at the newest stamp (fills 0..WINDOW_S at startup).
        right = max(snap["t_max"], WINDOW_S)
        for ax, _, _ in self._panels:
            ax.set_xlim(right - WINDOW_S, right)
        self.ax_pinger.set_xlim(right - WINDOW_S, right)

        # tight_layout is expensive and the layout barely changes between
        # frames: compute it once (subplot positions persist afterwards).
        if not self._layout_done:
            self.fig.tight_layout()
            self._layout_done = True

        # Land the PNG in the repo root, NOT the launch CWD. ros2 launch starts
        # nodes from $HOME, so the old cwd-relative path wrote to ~/sensors_plot.png
        # where nobody was looking. Prefer $ROBOSUB_DIR, then ~/RoboSub, then cwd.
        out_dir = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
        if not os.path.isdir(out_dir):
            out_dir = os.getcwd()
        out_path = os.path.join(out_dir, "sensors_plot.png")
        # dpi 70 (vs the 100 default) renders/encodes the PNG ~2x faster,
        # 1680x840 is still perfectly readable; compress_level 1 makes the
        # PNG encode cheap (file is a bit larger, nobody cares).
        try:
            self.fig.savefig(out_path, dpi=70,
                             pil_kwargs={"compress_level": 1})
        except TypeError:  # older matplotlib without pil_kwargs
            self.fig.savefig(out_path, dpi=70)
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
