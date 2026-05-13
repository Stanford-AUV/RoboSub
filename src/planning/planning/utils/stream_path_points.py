import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from control.utils.state import State
import os


class PathStreamer(Node):
    def __init__(self):
        super().__init__("path_streamer")
        self.create_subscription(Odometry, "/desired/pose", self.callback, 10)

        self.positions = []
        backend = plt.get_backend().lower()
        self.headless = backend == "agg" or not os.environ.get("DISPLAY")
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.image_path = os.environ.get(
            "PATH_STREAMER_IMAGE", "/tmp/path_streamer.png"
        )
        self.frame_count = 0
        if self.headless:
            self.get_logger().warning(
                f"Headless plotting backend detected; saving plot to {self.image_path}"
            )
        else:
            plt.ion()
        self._render_and_maybe_save()

    def _render_and_maybe_save(self, orientation=None):
        self.ax.cla()
        if self.positions:
            xs, ys, zs = zip(*self.positions)
            self.ax.plot(xs, ys, zs)
            if orientation is not None:
                fwd = Rotation.from_quat(
                    [orientation.x, orientation.y, orientation.z, orientation.w]
                ).apply([1, 0, 0])
                self.ax.quiver(*self.positions[-1], *fwd * 0.3, color="red")
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Desired Path")
        self.fig.canvas.draw_idle()
        if self.headless:
            self.fig.savefig(self.image_path, dpi=150)
        else:
            plt.pause(0.001)

    def callback(self, msg: Odometry):
        state = State.from_odometry_msg(msg)
        self.positions.append(state.position)

        self.frame_count += 1
        self._render_and_maybe_save(msg.pose.pose.orientation)


def main():
    rclpy.init()
    node = PathStreamer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
