import rclpy
from rclpy.node import Node
from msgs.msg import GeneratedPath 
from nav_msgs.msg import Odometry


class PathBroadcaster(Node):
    def __init__(self):
        super().__init__("path_broadcaster")

        self.create_subscription(GeneratedPath, "/generated_path", self.update_path, 10)
        self.publish_desired = self.create_publisher(Odometry, "/desired/pose", 10)

        self.current_path = None

        self.create_timer(1.0 / 60.0, self.publish_pose) 
    
    def update_path(self, msg):
        self.current_path = msg
        self.path_start_time = self.get_clock().now()
    
    def publish_pose(self):
        if self.current_path is None:
            return

        elapsed = (self.get_clock().now() - self.path_start_time).nanoseconds / 1e9
        duration = self.current_path.duration
        n = len(self.current_path.poses)

        if duration <= 0.0 or n == 0:
            return

        t = min(max(elapsed, 0.0), duration)
        index = int(t / duration * (n - 1))
        index = min(index, n - 1)

        odom = Odometry()
        odom.header = self.current_path.poses[index].header
        odom.pose.pose = self.current_path.poses[index].pose
        odom.twist.twist = self.current_path.twists[index]
        self.publish_desired.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PathBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
