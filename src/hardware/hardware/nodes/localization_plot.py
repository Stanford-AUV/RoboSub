"""Live EKF pose plot: 3D trajectory plus the six pose components vs time.

    ros2 run hardware localization_plot
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from transforms3d.euler import quat2euler

from hardware.utils import live_plot

POS_LIM = (-5, 6)
ANGLE_LIM = (-3.14, 3.14)
SERIES = ("x", "y", "z", "roll", "pitch", "yaw")


class LocalizationPlot(Node):
    def __init__(self):
        super().__init__("localization_plot")

        self.declare_parameter("update_period", 0.1)
        self._period = (
            self.get_parameter("update_period").get_parameter_value().double_value
        )

        # seconds=None: keep the whole run, otherwise the trajectory is a stub.
        self._window = live_plot.Window(SERIES, seconds=None)
        self._start = self.get_clock().now()
        self._last_draw = self._start

        fig = live_plot.make_figure()
        self._fig = fig
        self._ax_3d = live_plot.make_3d_axes(
            fig, 241, "Robot position trajectory", POS_LIM
        )
        self._panels = [
            (fig.add_subplot(242), "x", "X position (m)", "r-", POS_LIM),
            (fig.add_subplot(243), "y", "Y position (m)", "g-", POS_LIM),
            (fig.add_subplot(244), "z", "Z position (m)", "b-", POS_LIM),
            (fig.add_subplot(245), "roll", "Roll (rad)", "m-", ANGLE_LIM),
            (fig.add_subplot(246), "pitch", "Pitch (rad)", "c-", ANGLE_LIM),
            (fig.add_subplot(247), "yaw", "Yaw (rad)", "y-", ANGLE_LIM),
        ]
        live_plot.show(fig)

        self.create_subscription(
            Odometry, "/odometry/filtered", self._on_odom, 10
        )

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        now = self.get_clock().now()
        elapsed = (now - self._start).nanoseconds * 1e-9
        self._window.append(elapsed, (p.x, p.y, p.z, roll, pitch, yaw))

        if (now - self._last_draw).nanoseconds * 1e-9 >= self._period:
            self._redraw()
            self._last_draw = now

    def _redraw(self):
        w = self._window
        live_plot.draw_trajectory(
            self._ax_3d,
            w.column("x"),
            w.column("y"),
            w.column("z"),
            "Robot position trajectory",
            POS_LIM,
        )
        for ax, label, ylabel, color, ylim in self._panels:
            live_plot.draw_panel(ax, w.time, w.column(label), ylabel, color, ylim)
        live_plot.plt.tight_layout()
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationPlot()
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
