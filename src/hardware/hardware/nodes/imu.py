import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from msgs.msg import MTi200Data


class IMU(Node):

    def __init__(self):
        super().__init__("imu")
        self.subscription = self.create_subscription(
            MTi200Data,  # TODO make sure data type makes sense
            "/imu/data",
            self.listener_callback,
            10,
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(msg)
        self.get_logger().info('I heard: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    imu = IMU()
    rclpy.spin(imu)

    imu.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
