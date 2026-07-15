import rclpy
from rclpy.node import Node
from msgs.msg import SensorsStamped, PWMsStamped

import matplotlib

matplotlib.use("Agg")  # headless: write PNG to disk, no display forwarding needed
import matplotlib.pyplot as plt
import os
import numpy as np


class SensorsPlotSimplified(Node):
    """Simplified power/thermal/thruster health plot: five stacked subplots —
    current, voltage, power (V*I), temperature, and per-thruster PWM — written
    to sensors_plot_simplified.png. Current/voltage/temperature come from the
    Arduino sensor bundle (/arduino/sensors); PWMs come from the thrusters node
    (/pwms). Companion to the full localization sensors_plot node."""

    # savefig blocks the executor for ~1-2 s; queues must hold that much
    # backlog or DDS silently drops messages and the plot aliases (flat holds
    # turn into ramps). Keep this generous.
    QUEUE_DEPTH = 1000

    def __init__(self):
        super().__init__("sensors_plot_simplified")

        # Battery + thermal bundle (current, voltage, temperatures).
        self.arduino_sub = self.create_subscription(
            SensorsStamped, "/arduino/sensors", self.arduino_callback, self.QUEUE_DEPTH
        )
        # Per-thruster PWM commands published by the thrusters node.
        self.pwms_sub = self.create_subscription(
            PWMsStamped, "/pwms", self.pwms_callback, self.QUEUE_DEPTH
        )

        # figsize is tall: five time-series stacked in one column so current /
        # voltage / power / temp / pwm line up on a shared time axis.
        self.fig = plt.figure(figsize=(13, 16))
        self.ax_current = self.fig.add_subplot(511)
        self.ax_voltage = self.fig.add_subplot(512)
        self.ax_power = self.fig.add_subplot(513)
        self.ax_temp = self.fig.add_subplot(514)
        self.ax_pwm = self.fig.add_subplot(515)

        self.arduino_time = []
        self.current_history = []
        self.voltage_history = []
        self.power_history = []
        self.ext_temp_history = []
        self.int_temp1_history = []
        self.int_temp2_history = []

        self.pwm_time = []
        self.pwm_history = []  # each entry: list of per-thruster PWM ints

        # Redraw on a timer, NOT in the message callbacks: savefig takes ~1-2 s
        # and doing it inline starves the subscriptions and aliases the curves.
        self._t0 = None  # first header stamp seen; time axis is relative to it
        self.create_timer(2.0, self.update_plot)

    def _stamp(self, msg):
        """Seconds since start-of-run, from the message HEADER stamp (true
        sample time), not arrival time - arrival time smears samples that sat
        in the queue while the plotter was busy saving."""
        s = msg.header.stamp
        t = s.sec + s.nanosec * 1e-9
        if t == 0.0:  # unstamped publisher: fall back to node clock
            t = self.get_clock().now().nanoseconds * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def arduino_callback(self, msg: SensorsStamped):
        self.arduino_time.append(self._stamp(msg))
        self.current_history.append(msg.current)
        self.voltage_history.append(msg.voltage)
        self.power_history.append(msg.voltage * msg.current)
        self.ext_temp_history.append(msg.external_temperature)
        self.int_temp1_history.append(msg.internal_temperature1)
        self.int_temp2_history.append(msg.internal_temperature2)

    def pwms_callback(self, msg: PWMsStamped):
        self.pwm_time.append(self._stamp(msg))
        self.pwm_history.append(list(msg.pwms))

    def update_plot(self):
        for ax in (
            self.ax_current,
            self.ax_voltage,
            self.ax_power,
            self.ax_temp,
            self.ax_pwm,
        ):
            ax.cla()

        # ---- Current ----
        self.ax_current.set_title("Current (/arduino/sensors)")
        self.ax_current.plot(self.arduino_time, self.current_history, "r-", label="CURRENT")
        self.ax_current.set_ylabel("Current (A)")
        self.ax_current.grid(True)
        self.ax_current.legend(loc="upper right")

        # ---- Voltage ----
        self.ax_voltage.set_title("Voltage (/arduino/sensors)")
        self.ax_voltage.plot(self.arduino_time, self.voltage_history, "g-", label="VOLTAGE")
        self.ax_voltage.set_ylabel("Voltage (V)")
        self.ax_voltage.grid(True)
        self.ax_voltage.legend(loc="upper right")

        # ---- Power = V x I ----
        self.ax_power.set_title("Power (V x I)")
        self.ax_power.plot(self.arduino_time, self.power_history, "b-", label="POWER")
        self.ax_power.set_ylabel("Power (W)")
        self.ax_power.grid(True)
        self.ax_power.legend(loc="upper right")

        # ---- Temperature ----
        self.ax_temp.set_title("Temperature (/arduino/sensors)")
        self.ax_temp.plot(self.arduino_time, self.ext_temp_history, "b-", label="EXTERNAL")
        self.ax_temp.plot(self.arduino_time, self.int_temp1_history, "r-", label="INTERNAL 1")
        self.ax_temp.plot(self.arduino_time, self.int_temp2_history, "m-", label="INTERNAL 2")
        self.ax_temp.set_ylabel("Temperature (°C)")
        self.ax_temp.grid(True)
        self.ax_temp.legend(loc="upper right")

        # ---- Thruster PWM ----
        self.ax_pwm.set_title("Thruster PWM (/pwms)")
        if self.pwm_history:
            # Truncate every row to the common thruster count in case the very
            # first/last message is ragged, then plot one line per thruster.
            n = min(len(p) for p in self.pwm_history)
            if n > 0:
                arr = np.array([p[:n] for p in self.pwm_history])
                for i in range(n):
                    self.ax_pwm.plot(self.pwm_time, arr[:, i], label=f"T{i}")
        self.ax_pwm.axhline(1500, color="gray", ls="--", lw=0.8, label="neutral")
        self.ax_pwm.set_ylabel("PWM (µs)")
        self.ax_pwm.set_xlabel("Time (s)")
        self.ax_pwm.grid(True)
        self.ax_pwm.legend(loc="upper right", ncol=3, fontsize="small")

        plt.tight_layout()
        # Land the PNG in the repo root, NOT the launch CWD. ros2 launch starts
        # nodes from $HOME, so a cwd-relative path wrote to ~/sensors_plot.png
        # where nobody was looking. Prefer $ROBOSUB_DIR, then ~/RoboSub, then cwd.
        out_dir = os.environ.get("ROBOSUB_DIR") or os.path.expanduser("~/RoboSub")
        if not os.path.isdir(out_dir):
            out_dir = os.getcwd()
        # Output filename overridable via $SENSORS_PLOT_OUT (teleop_remote.sh
        # sets it to joystick.png).
        out_name = os.environ.get("SENSORS_PLOT_OUT", "sensors_plot_simplified.png")
        out_path = os.path.join(out_dir, out_name)
        self.fig.savefig(out_path)
        if not getattr(self, "_logged_out_path", False):
            self.get_logger().info(f"writing simplified sensors plot to {out_path}")
            self._logged_out_path = True


def main(args=None):
    rclpy.init(args=args)
    node = SensorsPlotSimplified()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        plt.close("all")


if __name__ == "__main__":
    main()
