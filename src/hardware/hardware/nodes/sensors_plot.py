import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from msgs.msg import DVLData


class SensorsPlot(Node):
    def __init__(self):
        super().__init__("sensors_plot")

        # Create subscribers for IMU and DVL
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, 10)
        self.dvl_sub = self.create_subscription(DVLData, "/dvl", self.dvl_callback, 10)

        # Initialize plot
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(12, 8))

        # Create subplots
        self.ax_imu_accel = self.fig.add_subplot(221)
        self.ax_imu_gyro = self.fig.add_subplot(222)
        self.ax_dvl_vel = self.fig.add_subplot(223)

        # Set titles
        self.ax_imu_accel.set_title("IMU Acceleration")
        self.ax_imu_gyro.set_title("IMU Angular Velocity")
        self.ax_dvl_vel.set_title("DVL Velocity")

        # Initialize data history
        self.time_history = []
        self.start_time = self.get_clock().now()

        # IMU data history
        self.accel_x_history = []
        self.accel_y_history = []
        self.accel_z_history = []
        self.gyro_x_history = []
        self.gyro_y_history = []
        self.gyro_z_history = []

        # DVL data history
        self.vel_x_history = []
        self.vel_y_history = []
        self.vel_z_history = []

        # Last received values for DVL
        self.last_vel_x = 0.0
        self.last_vel_y = 0.0
        self.last_vel_z = 0.0

        # Plot update rate (in seconds)
        self.update_period = 0.1
        self.last_plot_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now()
        time_sec = (current_time - self.start_time).nanoseconds * 1e-9

        # Extract IMU data
        self.accel_x_history.append(msg.linear_acceleration.x)
        self.accel_y_history.append(msg.linear_acceleration.y)
        self.accel_z_history.append(msg.linear_acceleration.z)
        self.gyro_x_history.append(msg.angular_velocity.x)
        self.gyro_y_history.append(msg.angular_velocity.y)
        self.gyro_z_history.append(msg.angular_velocity.z)

        # Add DVL data using last known values
        self.vel_x_history.append(self.last_vel_x)
        self.vel_y_history.append(self.last_vel_y)
        self.vel_z_history.append(self.last_vel_z)

        self.time_history.append(time_sec)

        # Update plot periodically
        if (
            current_time - self.last_plot_time
        ).nanoseconds * 1e-9 >= self.update_period:
            self.update_plot()
            self.last_plot_time = current_time

    def dvl_callback(self, msg: DVLData):
        # Store latest DVL values
        self.last_vel_x = msg.velocity.mean.x
        self.last_vel_y = msg.velocity.mean.y
        self.last_vel_z = msg.velocity.mean.z

    def update_plot(self):
        # Clear all plots
        self.ax_imu_accel.cla()
        self.ax_imu_gyro.cla()
        self.ax_dvl_vel.cla()

        # Plot IMU acceleration
        self.ax_imu_accel.plot(self.time_history, self.accel_x_history, "r-", label="X")
        self.ax_imu_accel.plot(self.time_history, self.accel_y_history, "g-", label="Y")
        self.ax_imu_accel.plot(self.time_history, self.accel_z_history, "b-", label="Z")
        self.ax_imu_accel.set_xlabel("Time (s)")
        self.ax_imu_accel.set_ylabel("Acceleration (m/s²)")
        self.ax_imu_accel.grid(True)
        self.ax_imu_accel.legend()

        # Plot IMU angular velocity
        self.ax_imu_gyro.plot(self.time_history, self.gyro_x_history, "r-", label="X")
        self.ax_imu_gyro.plot(self.time_history, self.gyro_y_history, "g-", label="Y")
        self.ax_imu_gyro.plot(self.time_history, self.gyro_z_history, "b-", label="Z")
        self.ax_imu_gyro.set_xlabel("Time (s)")
        self.ax_imu_gyro.set_ylabel("Angular Velocity (rad/s)")
        self.ax_imu_gyro.grid(True)
        self.ax_imu_gyro.legend()

        # Plot DVL velocity
        self.ax_dvl_vel.plot(self.time_history, self.vel_x_history, "r-", label="X")
        self.ax_dvl_vel.plot(self.time_history, self.vel_y_history, "g-", label="Y")
        self.ax_dvl_vel.plot(self.time_history, self.vel_z_history, "b-", label="Z")
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
