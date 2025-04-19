import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib
import numpy as np
from transforms3d.euler import quat2euler

matplotlib.use("TkAgg")  # Use a GUI backend
import matplotlib.pyplot as plt

# correction = np.array([0, 0, -9.80665]) - np.array(
#     [-0.5094561923194577, 0.16367032612896282, -9.79881680978311]
# )
correction = np.array([0, 0, 0])


class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")
        # Subscribe to the IMU topic (ORS publishes under "/imu")
        self.imu_sub = self.create_subscription(Imu, "imu", self.imu_callback, 10)

        # Data histories for a sliding 10-second window
        self.time_history = []
        self.accel_x_history = []
        self.accel_y_history = []
        self.accel_z_history = []
        self.start_time = self.get_clock().now()

        # Setup interactive plotting
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title("IMU Acceleration")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Acceleration (m/s²)")
        self.ax.grid(True)
        plt.show(block=False)

        # Create a timer to update the plot regularly
        self.plot_timer = self.create_timer(0.1, self.update_plot)

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        time_sec = (current_time - self.start_time).nanoseconds * 1e-9

        # Append new IMU acceleration data
        self.time_history.append(time_sec)
        # qx = msg.orientation.x
        # qy = msg.orientation.y
        # qz = msg.orientation.z
        # qw = msg.orientation.w
        # roll, pitch, yaw = quat2euler([qw, qx, qy, qz])
        x, y, z = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            # roll,
            # pitch,
            # yaw,
            # msg.angular_velocity.x,
            # msg.angular_velocity.y,
            # msg.angular_velocity.z,
        )
        x, y, z = x + correction[0], y + correction[1], z + correction[2]
        self.accel_x_history.append(x)
        self.accel_y_history.append(y)
        self.accel_z_history.append(z)

    def update_plot(self):
        # Clear and redraw the plot with current data
        self.ax.cla()
        self.ax.plot(self.time_history, self.accel_x_history, "r-", label="Accel X")
        self.ax.plot(self.time_history, self.accel_y_history, "g-", label="Accel Y")
        self.ax.plot(self.time_history, self.accel_z_history, "b-", label="Accel Z")

        if self.time_history:
            avg_x = sum(self.accel_x_history) / len(self.accel_x_history)
            avg_y = sum(self.accel_y_history) / len(self.accel_y_history)
            avg_z = sum(self.accel_z_history) / len(self.accel_z_history)
            self.ax.axhline(
                avg_x, color="r", linestyle="--", label=f"Avg X: {avg_x:.2f}"
            )
            self.ax.axhline(
                avg_y, color="g", linestyle="--", label=f"Avg Y: {avg_y:.2f}"
            )
            self.ax.axhline(
                avg_z, color="b", linestyle="--", label=f"Avg Z: {avg_z:.2f}"
            )
            print(f"Average: {avg_x} {avg_y} {avg_z}")

        self.ax.set_title("IMU Acceleration")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Acceleration (m/s²)")
        self.ax.grid(True)
        # Only add the legend if there is data
        if self.time_history:
            self.ax.legend()
        plt.tight_layout()
        # Use canvas draw and flush_events for non-blocking update
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = SensorsPlot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close("all")


if __name__ == "__main__":
    main()
