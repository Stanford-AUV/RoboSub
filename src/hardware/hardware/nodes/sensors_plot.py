import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
import matplotlib.pyplot as plt


class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")

        self.accel_sub = self.create_subscription(
            Imu, "/accel", self.accel_callback, 10
        )
        self.angular_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/angular", self.angular_callback, 10
        )
        self.velocity_sub = self.create_subscription(
            TwistWithCovarianceStamped, "/velocity", self.velocity_callback, 10
        )

        plt.ion()
        self.fig = plt.figure(figsize=(12, 8))

        self.ax_imu_accel = self.fig.add_subplot(221)
        self.ax_imu_gyro = self.fig.add_subplot(222)
        self.ax_dvl_vel = self.fig.add_subplot(223)

        self.ax_imu_accel.set_title("IMU Acceleration")
        self.ax_imu_gyro.set_title("IMU Angular Velocity")
        self.ax_dvl_vel.set_title("DVL Velocity")

        self.start_time = self.get_clock().now()

        self.accel_time = []
        self.accel_x_history = []
        self.accel_y_history = []
        self.accel_z_history = []

        self.gyro_time = []
        self.gyro_x_history = []
        self.gyro_y_history = []
        self.gyro_z_history = []

        self.vel_time = []
        self.vel_x_history = []
        self.vel_y_history = []
        self.vel_z_history = []

        self.update_period = 0.1
        self.last_plot_time = self.get_clock().now()

    def _elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def _maybe_update_plot(self):
        now = self.get_clock().now()
        if (now - self.last_plot_time).nanoseconds * 1e-9 >= self.update_period:
            self.update_plot()
            self.last_plot_time = now

    def accel_callback(self, msg: Imu):
        self.accel_time.append(self._elapsed())
        self.accel_x_history.append(msg.linear_acceleration.x)
        self.accel_y_history.append(msg.linear_acceleration.y)
        self.accel_z_history.append(msg.linear_acceleration.z)
        self._maybe_update_plot()

    def angular_callback(self, msg: TwistWithCovarianceStamped):
        self.gyro_time.append(self._elapsed())
        self.gyro_x_history.append(msg.twist.twist.angular.x)
        self.gyro_y_history.append(msg.twist.twist.angular.y)
        self.gyro_z_history.append(msg.twist.twist.angular.z)
        self._maybe_update_plot()

    def velocity_callback(self, msg: TwistWithCovarianceStamped):
        self.vel_time.append(self._elapsed())
        self.vel_x_history.append(msg.twist.twist.linear.x)
        self.vel_y_history.append(msg.twist.twist.linear.y)
        self.vel_z_history.append(msg.twist.twist.linear.z)
        self._maybe_update_plot()

    def update_plot(self):
        # Clear all plots
        self.ax_imu_accel.cla()
        self.ax_imu_gyro.cla()
        self.ax_dvl_vel.cla()

        self.ax_imu_accel.plot(self.accel_time, self.accel_x_history, "r-", label="X")
        self.ax_imu_accel.plot(self.accel_time, self.accel_y_history, "g-", label="Y")
        self.ax_imu_accel.plot(self.accel_time, self.accel_z_history, "b-", label="Z")
        self.ax_imu_accel.set_xlabel("Time (s)")
        self.ax_imu_accel.set_ylabel("Acceleration (m/s²)")
        self.ax_imu_accel.grid(True)
        self.ax_imu_accel.legend()

        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_x_history, "r-", label="X")
        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_y_history, "g-", label="Y")
        self.ax_imu_gyro.plot(self.gyro_time, self.gyro_z_history, "b-", label="Z")
        self.ax_imu_gyro.set_xlabel("Time (s)")
        self.ax_imu_gyro.set_ylabel("Angular Velocity (rad/s)")
        self.ax_imu_gyro.grid(True)
        self.ax_imu_gyro.legend()

        self.ax_dvl_vel.plot(self.vel_time, self.vel_x_history, "r-", label="X")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_y_history, "g-", label="Y")
        self.ax_dvl_vel.plot(self.vel_time, self.vel_z_history, "b-", label="Z")
        self.ax_dvl_vel.set_xlabel("Time (s)")
        self.ax_dvl_vel.set_ylabel("Velocity (m/s)")
        self.ax_dvl_vel.grid(True)
        self.ax_dvl_vel.legend()

        # Adjust layout and update display
        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)


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
