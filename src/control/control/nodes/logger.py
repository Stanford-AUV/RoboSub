import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import time


class Logger(Node):
    def __init__(self):
        super().__init__("logger")
        self.create_subscription(Odometry, "odometry", self.log_data, 10)

        # Initialize data storage
        self.time_data = []
        self.position_data = {"x": [], "y": [], "z": []}
        self.velocity_data = {"vx": [], "vy": [], "vz": []}
        self.start_time = time.time()

        # Set up Matplotlib figure and axes
        self.fig, self.axs = plt.subplots(2, 1, figsize=(10, 8))
        self.axs[0].set_title("Position vs Time")
        self.axs[1].set_title("Velocity vs Time")

        self.axs[0].set_ylabel("Position (m)")
        self.axs[1].set_ylabel("Velocity (m/s)")
        self.axs[1].set_xlabel("Time (s)")

        self.lines = {
            "x": self.axs[0].plot([], [], label="x")[0],
            "y": self.axs[0].plot([], [], label="y")[0],
            "z": self.axs[0].plot([], [], label="z")[0],
            "vx": self.axs[1].plot([], [], label="vx")[0],
            "vy": self.axs[1].plot([], [], label="vy")[0],
            "vz": self.axs[1].plot([], [], label="vz")[0],
        }

        for ax in self.axs:
            ax.legend()

        # Create a ROS 2 timer to periodically update the plot
        self.timer = self.create_timer(0.01, self.update_plot)

        # Show the plot
        plt.ion()  # Interactive mode for real-time updates
        self.fig.show()

    def log_data(self, msg: Odometry):
        """Callback to log odometry data."""
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)

        self.position_data["x"].append(msg.pose.pose.position.x)
        self.position_data["y"].append(msg.pose.pose.position.y)
        self.position_data["z"].append(msg.pose.pose.position.z)

        self.velocity_data["vx"].append(msg.twist.twist.linear.x)
        self.velocity_data["vy"].append(msg.twist.twist.linear.y)
        self.velocity_data["vz"].append(msg.twist.twist.linear.z)

    def update_plot(self):
        """Update plot with the latest data."""
        # Update the line data
        for key, line in self.lines.items():
            if key in self.position_data:
                line.set_data(self.time_data, self.position_data[key])
            elif key in self.velocity_data:
                line.set_data(self.time_data, self.velocity_data[key])

        # Adjust plot limits dynamically
        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()

        # Redraw the figure
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    logger = Logger()
    try:
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("Shutting down Logger.")
    finally:
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
