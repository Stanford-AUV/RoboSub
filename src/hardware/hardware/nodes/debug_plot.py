"""One bench plot node for any of the PRESETS below.

    ros2 run hardware debug_plot --ros-args -p preset:=dvl_velocity

Replaces the old per-sensor imu_plot / dvl_plot nodes: they differed only in
topic, message type and which fields to pull. The plotting itself lives in
hardware/utils/live_plot.py.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from msgs.msg import DVLData
from hardware.utils import live_plot

# preset -> (topic, message type, title, y-axis label, series labels,
#            sample extractor, show per-series means)
PRESETS = {
    "imu_accel": (
        "imu",
        Imu,
        "IMU acceleration",
        "Acceleration (m/s^2)",
        ("accel X", "accel Y", "accel Z"),
        lambda m: (
            m.linear_acceleration.x,
            m.linear_acceleration.y,
            m.linear_acceleration.z,
        ),
        True,
    ),
    "dvl_velocity": (
        "dvl",
        DVLData,
        "DVL velocity",
        "Velocity (m/s)",
        ("vel X", "vel Y", "vel Z"),
        lambda m: (m.velocity.mean.x, m.velocity.mean.y, m.velocity.mean.z),
        False,
    ),
}


class DebugPlot(Node):
    def __init__(self):
        super().__init__("debug_plot")

        self.declare_parameter("preset", "imu_accel")
        self.declare_parameter("seconds", 10.0)
        name = self.get_parameter("preset").get_parameter_value().string_value
        if name not in PRESETS:
            raise SystemExit(
                f"unknown preset {name!r}. Available: {sorted(PRESETS)}"
            )
        seconds = self.get_parameter("seconds").get_parameter_value().double_value

        topic, msg_type, title, ylabel, labels, extract, means = PRESETS[name]
        self._title, self._ylabel, self._means = title, ylabel, means
        self._extract = extract
        self._window = live_plot.Window(labels, seconds=seconds)
        self._start = self.get_clock().now()

        self.create_subscription(msg_type, topic, self._on_msg, 10)
        self._fig, self._ax = live_plot.make_axes(
            f"{title} (last {seconds:.0f} s)", ylabel
        )
        self.create_timer(0.1, self._redraw)
        self.get_logger().info(f"plotting {name} from {topic}")

    def _on_msg(self, msg):
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        self._window.append(elapsed, self._extract(msg))

    def _redraw(self):
        live_plot.redraw(
            self._fig,
            self._ax,
            self._window,
            f"{self._title} (last {self._window.seconds:.0f} s)",
            self._ylabel,
            show_means=self._means,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DebugPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        live_plot.plt.close("all")


if __name__ == "__main__":
    main()
