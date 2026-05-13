#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class ZeroWrenchPublisher(Node):
    def __init__(self):
        super().__init__('zero_wrench_publisher')
        # Publish on the same 'wrench' topic that thrust_generator listens to
        self._pub = self.create_publisher(WrenchStamped, 'wrench', 10)

        # Send zeros at 10 Hz (adjust if you need a different rate)
        timer_period = 0.1
        self.create_timer(timer_period, self._on_timer)

    def _on_timer(self):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # By default, all fields of WrenchStamped.wrench are zero,
        # so this publishes zero force & zero torque.
        self._pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ZeroWrenchPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
