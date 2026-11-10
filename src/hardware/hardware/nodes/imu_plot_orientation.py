"""Live 3D view of the vehicle's orientation as reported on /imu/orientation.

Draws the base_link X/Y/Z axes inside a unit sphere.

    ros2 run hardware imu_plot_orientation

The quaternion is plotted as published: hardware/imu already remounts the
sensor into base_link via R_sensor_to_base from sensors.yaml (identity since
the mount rotation was burned into the MTi), so applying any further rotation
here would disagree with what the EKF fuses.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

from hardware.utils import live_plot

AXES_LIM = (-1.2, 1.2)


class ImuOrientationPlot(Node):
    def __init__(self):
        super().__init__("imu_orientation_plot")

        self.declare_parameter("topic", "/imu/orientation")
        topic = self.get_parameter("topic").get_parameter_value().string_value

        self._rotation = None
        self._arrows = None

        fig = live_plot.make_figure(figsize=(10, 10))
        self._fig = fig
        self._ax = live_plot.make_3d_axes(fig, 111, "IMU orientation", AXES_LIM)
        live_plot.draw_unit_sphere(self._ax)  # static reference, drawn once
        live_plot.show(fig)

        self.create_subscription(Imu, topic, self._on_imu, 10)
        self.create_timer(0.1, self._redraw)
        self.get_logger().info(f"plotting orientation from {topic}")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._rotation = R.from_quat([q.x, q.y, q.z, q.w])

    def _redraw(self):
        if self._rotation is None:
            return
        self._arrows = live_plot.draw_frame_axes(
            self._ax, self._rotation.as_matrix(), previous=self._arrows
        )
        self._ax.legend()
        self._fig.canvas.draw()
        self._fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = ImuOrientationPlot()
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
