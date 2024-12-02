import numpy as np
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

class SimTester(Node):

    def __init__(self):
        super().__init__('sim_tester')
        self.path_publisher = self.create_publisher(Odometry, 'path', 10)
        #self.timer = self.create_timer(1.0, self.publish_path)  # Timer to publish every 1 second
        self.x_position = 10.0  # Initial x position

    def publish_path(self):
        msg = Odometry()
        msg.pose.pose.position.x = self.x_position
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Publishing: x = {msg.pose.pose.position.x}')
        self.path_publisher.publish(msg)

        # Increment x position for the next publish
        self.x_position += 0.0

def main(args=None):
    rclpy.init(args=args)
    test_path = SimTester()
    rclpy.spin(test_path)
    test_path.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()