import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from msgs.msg import MTi200Data

from geometry_msgs.msg import Vector3Stamped

class IMU(Node):

    def __init__(self):
        super().__init__("imu")
        self.imu_subscription = self.create_subscription(
            Imu,  # TODO make sure data type makes sense
            "/imu/data",
            self.imu_listener_callback,
            10,
        )

        self._imu_pub = self.create_publisher(Imu, "imu", 10)


    def imu_listener_callback(self, msg):
        self.get_logger().info(f"{msg}")
        self._imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    imu = IMU()
    rclpy.spin(imu)
    imu.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
