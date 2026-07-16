import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from msgs.msg import PingerStamped, SensorsStamped
from hardware.pinger import detector as pinger_detector
from hardware.utils import dashboard

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
# Aggressive path simplification: rasterizing the line paths dominates the
# frame time on the Orin; visually lossless for time-series at this size.
matplotlib.rcParams["path.simplify"] = True
matplotlib.rcParams["path.simplify_threshold"] = 1.0
import matplotlib.pyplot as plt
import bisect
import collections
import math
import os
import threading
import time
from types import SimpleNamespace

import numpy as np

# The plot shows a sliding window of the last WINDOW_S seconds; samples
# older than that are pruned at snapshot time (they can never be shown
# again), so storage stays ~window-sized regardless of run length.
WINDOW_S = 20.0
# Backstop cap in case pruning falls behind (e.g. a stamp glitch).
HISTORY_MAX = 50_000
# Samples scrolling out of the window are archived at this coarse spacing
# so the PNG can show the WHOLE run from t=0 (2 Hz x hours = tiny).
ARCHIVE_DT = 0.5
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

        # Row 1: inputs + desired setpoints.
        # Row 2: EKF outputs.
        # Row 3: power electronics + temperatures + pinger.
        self.ax_imu_in = self.fig.add_subplot(341)
        self.ax_dvl_vel = self.fig.add_subplot(342)
        self.ax_des_pos = self.fig.add_subplot(343)
        self.ax_des_rot = self.fig.add_subplot(344)
        self.ax_ekf_rot = self.fig.add_subplot(345)
        self.ax_ekf_accel = self.fig.add_subplot(346)
        self.ax_ekf_vel = self.fig.add_subplot(347)
        self.ax_ekf_pos = self.fig.add_subplot(348)
        self.ax_current = self.fig.add_subplot(349)
        self.ax_voltage = self.fig.add_subplot(3, 4, 10)
        self.ax_temp = self.fig.add_subplot(3, 4, 11)
        # Pinger: 4 hydrophone levels + front/back decision.
        self.ax_pinger = self.fig.add_subplot(3, 4, 12)

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

        # Coarse full-run archive fed by the pruning in _snapshot_locked:
        # the web dashboard shows the sliding window, the PNG shows
        # archive + window = the whole run from t=0.
        self._arch = {}
        for tk, sks in self._groups.items():
            self._arch[tk] = []
            for sk in sks:
                self._arch[sk] = []
        self._arch_levels = [[], [], [], []]
        self._arch_last = {tk: -1e9 for tk in self._groups}

        # ---- Persistent artists: titles/labels/legends/lines are created
        # ONCE here; each frame only calls set_data on the lines. Clearing
        # and re-plotting 12 panels (cla + plot + legend) cost ~1.5 s/frame
        # on the Orin; updating line data is ~10x cheaper.
        # panel() also records a plain spec of itself so the web dashboard
        # renders exactly the same panels from /data.json.
        self._panels = []
        self._web_panels = []
        self._unwrap_keys = set()
        web_colors = {"r": "#f55", "g": "#5d5", "b": "#69f",
                      "m": "#d6d", "c": "#4dd"}

        def panel(ax, title, ylabel, time_key, series):
            ax.set_title(title)
            ax.set_ylabel(ylabel)
            ax.set_xlabel("Time (s)")
            ax.grid(True)
            lines = [(ax.plot([], [], style, label=label)[0], key, unwrap)
                     for key, style, label, unwrap in series]
            ax.legend(loc="upper left", fontsize="x-small")
            self._panels.append((ax, time_key, lines))
            self._web_panels.append({
                "title": title, "time": time_key,
                "series": [{"key": key, "label": label,
                            "color": web_colors[style[0]]}
                           for key, style, label, unwrap in series],
            })
            self._unwrap_keys.update(
                key for key, _, _, unwrap in series if unwrap)

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
        panel(self.ax_ekf_rot, "EKF Rotation (/odometry/filtered)",
              "Rotation (rad)", "rot_time",
              [("rot_x_history", "r-", "ROLL", True),
               ("rot_y_history", "g-", "PITCH", True),
               ("rot_z_history", "b-", "YAW", True)])
        panel(self.ax_ekf_accel, "EKF Acceleration (/accel/filtered)",
              "Acceleration (m/s²)", "ekf_accel_time",
              [("ekf_accel_x_history", "r-", "X", False),
               ("ekf_accel_y_history", "g-", "Y", False),
               ("ekf_accel_z_history", "b-", "Z", False)])
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
        panel(self.ax_current, "Current (/arduino/sensors)",
              "Current (A)", "arduino_time",
              [("current_history", "r-", "CURRENT", False)])
        panel(self.ax_voltage, "Voltage (/arduino/sensors)",
              "Voltage (V)", "arduino_time",
              [("voltage_history", "g-", "VOLTAGE", False)])
        panel(self.ax_temp, "Temperature (/arduino/sensors)",
              "Temperature (°C)", "arduino_time",
              [("ext_temp_history", "b-", "EXTERNAL", False),
               ("int_temp1_history", "r-", "INTERNAL 1", False),
               ("int_temp2_history", "m-", "INTERNAL 2", False)])

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

        # Plots land under plots/<session>/: full.png (whole run, from 0)
        # plus window/<k>.png for every completed WINDOW_S-second chunk.
        root = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
        if not os.path.isdir(root):
            root = os.getcwd()
        self._plots_dir = os.path.join(
            root, "plots", time.strftime("%Y_%m_%d_%H_%M_%S"))
        os.makedirs(os.path.join(self._plots_dir, "window"), exist_ok=True)
        self._chunks_done = 0

        self._drawing = False
        self._layout_done = False
        self._t0 = None  # first header stamp seen; time axis is relative to it

        # Live web dashboard: the browser polls /data.json ~5x/s and does
        # ALL the rendering, so it updates near-instantly while the Orin
        # only serializes the window. The PNG below stays as a slow,
        # crash-surviving record (and for anyone without a browser).
        page = dashboard.build_page(self._web_panels, {
            "title": f"Pinger levels @ {pinger_detector.targetFrequency:.0f} Hz",
            "time": "pinger_time",
            "labels": [f"ch{ch} ({'front' if ch in pinger_detector.FRONT_CHANNELS else 'back'})"
                       for ch in range(4)],
            "colors": ["#69f", "#f55", "#d6d", "#4dd"],
            "threshold": pinger_detector.baseThreshold,
        })
        self._http = dashboard.start_server(
            page, self._web_data, self.get_logger().info)

        self.create_timer(5.0, self.request_draw)

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
            # Completed WINDOW_S chunks since the last render (usually 0 or
            # 1; more only if drawing stalled). Snapshot BEFORE pruning --
            # _snapshot_locked never prunes past an unrendered chunk.
            chunks = []
            t_max = max(
                (getattr(self, tk)[-1] for tk in self._groups
                 if getattr(self, tk)),
                default=0.0,
            )
            while t_max >= (self._chunks_done + 1) * WINDOW_S:
                k = self._chunks_done
                chunks.append((k, self._chunk_snapshot_locked(k)))
                self._chunks_done += 1
            win = self._snapshot_locked()
            # Full-run frame = coarse archive + the current window.
            snap = {"t_max": win["t_max"]}
            for key, v in win.items():
                if key in ("t_max", "pinger_levels_history"):
                    continue
                snap[key] = _decimate(self._arch.get(key, []) + v)
            snap["pinger_levels_history"] = [
                _decimate(a + w) for a, w in
                zip(self._arch_levels, win["pinger_levels_history"])
            ]
        self._drawing = True
        threading.Thread(target=self._draw, args=(snap, chunks),
                         daemon=True).start()

    def _chunk_snapshot_locked(self, k):
        """Full-resolution copy of chunk k = [k*WINDOW_S, (k+1)*WINDOW_S)."""
        lo, hi = k * WINDOW_S, (k + 1) * WINDOW_S
        snap = {"t_max": hi}
        for tk, series_keys in self._groups.items():
            times = list(getattr(self, tk))
            i0 = bisect.bisect_left(times, lo)
            i1 = bisect.bisect_left(times, hi)
            snap[tk] = _decimate(times[i0:i1])
            for sk in series_keys:
                snap[sk] = _decimate(list(getattr(self, sk))[i0:i1])
            if tk == "pinger_time":
                snap["pinger_levels_history"] = [
                    _decimate(list(d)[i0:i1])
                    for d in self.pinger_levels_history
                ]
        return snap

    def _web_data(self):
        """Payload for the dashboard's /data.json (called from the HTTP
        server threads). Same window/pruning as the PNG."""
        with self._lock:
            snap = self._snapshot_locked()
        series = {}
        for key, val in snap.items():
            if key in ("t_max", "pinger_levels_history"):
                continue
            if key in self._unwrap_keys:
                val = np.unwrap(val).tolist()
            series[key] = [round(float(v), 4) for v in val]
        front = snap["pinger_front_history"]
        return {
            "t": snap["t_max"],
            "series": series,
            "pinger_levels": [[round(float(v), 4) for v in lv]
                              for lv in snap["pinger_levels_history"]],
            "front": bool(front[-1]) if front else None,
        }

    def _snapshot_locked(self):
        """Prune + window + decimate every series; caller holds _lock.
        Never prunes into a chunk that window/<k>.png hasn't rendered."""
        t_max = max(
            (getattr(self, tk)[-1] for tk in self._groups
             if getattr(self, tk)),
            default=0.0,
        )
        t_left = min(t_max - WINDOW_S, self._chunks_done * WINDOW_S)
        snap = {"t_max": t_max}
        for tk, series_keys in self._groups.items():
            times = getattr(self, tk)
            cols = [getattr(self, sk) for sk in series_keys]
            if tk == "pinger_time":
                cols = cols + self.pinger_levels_history
            # Prune everything that scrolled out of the window, keeping a
            # coarse sample of it in the full-run archive for full.png.
            while times and times[0] < t_left:
                t0 = times.popleft()
                vals = [c.popleft() for c in cols]
                if t0 - self._arch_last[tk] >= ARCHIVE_DT:
                    self._arch_last[tk] = t0
                    self._arch[tk].append(t0)
                    n = len(series_keys)
                    for sk, v in zip(series_keys, vals[:n]):
                        self._arch[sk].append(v)
                    for lst, v in zip(self._arch_levels, vals[n:]):
                        lst.append(v)
            snap[tk] = _decimate(times)
            for sk, c in zip(series_keys, cols):
                snap[sk] = _decimate(c)
        snap["pinger_levels_history"] = [
            _decimate(d) for d in self.pinger_levels_history
        ]
        return snap

    def _draw(self, snap, chunks):
        try:
            for k, csnap in chunks:
                self._apply_data(csnap)
                self._set_xlim(k * WINDOW_S, (k + 1) * WINDOW_S)
                self._save(os.path.join(self._plots_dir, "window",
                                        f"{k}.png"))
            self._apply_data(snap)
            # full.png is the whole-run record: always from t=0 (the web
            # dashboard is the one that scrolls a sliding window).
            self._set_xlim(0.0, max(snap["t_max"], WINDOW_S))
            self._save(os.path.join(self._plots_dir, "full.png"))
        except Exception as e:  # a draw glitch must never kill the node
            self.get_logger().error(f"plot draw failed: {e}")
        finally:
            self._drawing = False

    def _apply_data(self, snap):
        # Update line data in place (artists are created once in __init__).
        for ax, time_key, lines in self._panels:
            t = snap[time_key]
            for ln, key, unwrap in lines:
                y = snap[key]
                ln.set_data(t, np.unwrap(y) if unwrap else y)
            if t:  # rescale y to the data now shown
                ax.relim()
                ax.autoscale_view(scalex=False)

        t = snap["pinger_time"]
        for ln, y in zip(self._pinger_lines, snap["pinger_levels_history"]):
            ln.set_data(t, y)
        front = snap["pinger_front_history"]
        self._pinger_text.set_text(
            ("FRONT" if front[-1] else "BACK") if front else "")

    def _set_xlim(self, lo, hi):
        for ax, _, _ in self._panels:
            ax.set_xlim(lo, hi)
        self.ax_pinger.set_xlim(lo, hi)

    def _save(self, path):
        # tight_layout is expensive and the layout barely changes between
        # frames: compute it once (subplot positions persist afterwards).
        if not self._layout_done:
            self.fig.tight_layout()
            self._layout_done = True
        # dpi 70 (vs the 100 default) renders/encodes the PNG ~2x faster,
        # 1680x840 is still perfectly readable; compress_level 1 makes the
        # PNG encode cheap (file is a bit larger, nobody cares).
        try:
            self.fig.savefig(path, dpi=70, pil_kwargs={"compress_level": 1})
        except TypeError:  # older matplotlib without pil_kwargs
            self.fig.savefig(path, dpi=70)
        if not getattr(self, "_logged_out_path", False):
            self.get_logger().info(
                f"writing sensor plots under {self._plots_dir}")
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
