import rclpy
from rclpy.node import Node
import matplotlib
from msgs.msg import DVLData

matplotlib.use("TkAgg")  # Use a GUI backend
import matplotlib.pyplot as plt


class DvlPlot(Node):
    def __init__(self):
        super().__init__("dvl_plot")
        # Subscribe to the DVL topic (DVL publishes under "/dvl")
        self.dvl_sub = self.create_subscription(DVLData, "dvl", self.dvl_callback, 10)

        # Data histories for a sliding 10-second window
        self.time_history = []
        self.vel_x_history = []
        self.vel_y_history = []
        self.vel_z_history = []
        self.start_time = self.get_clock().now()

        # Setup interactive plotting
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_title("DVL Velocity (Last 10 seconds)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (m/s)")
        self.ax.grid(True)
        plt.show(block=False)

        # Create a timer to update the plot regularly
        self.plot_timer = self.create_timer(0.1, self.update_plot)

    def dvl_callback(self, msg: DVLData):
        current_time = self.get_clock().now()
        time_sec = (current_time - self.start_time).nanoseconds * 1e-9

        # Append new DVL velocity data
        self.time_history.append(time_sec)
        dvl_velocity = msg.velocity.mean
        self.vel_x_history.append(dvl_velocity.x)
        self.vel_y_history.append(dvl_velocity.y)
        self.vel_z_history.append(dvl_velocity.z)

        # Remove data older than 10 seconds
        while self.time_history and self.time_history[0] < time_sec - 10:
            self.time_history.pop(0)
            self.vel_x_history.pop(0)
            self.vel_y_history.pop(0)
            self.vel_z_history.pop(0)

    def update_plot(self):
        # Clear and redraw the plot with current data
        self.ax.cla()
        self.ax.plot(self.time_history, self.vel_x_history, "r-", label="Vel X")
        self.ax.plot(self.time_history, self.vel_y_history, "g-", label="Vel Y")
        self.ax.plot(self.time_history, self.vel_z_history, "b-", label="Vel Z")
        self.ax.set_title("DVL Velocity (Last 10 seconds)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Velocity (m/s)")
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
    node = DvlPlot()
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
