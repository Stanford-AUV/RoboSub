import numpy as np


from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node


class testPath(Node):

    def __init__(self):
        super().__init__('test_path')

        self.path_publisher = self.create_publisher(Odometry, 'path', 10)

        

    def publish_path(self):
        msg = Odometry()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        self.path_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    test_path = testPath()
    test_path.publish_path()

    rclpy.spin(test_path)

    test_path.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()