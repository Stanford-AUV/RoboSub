import rclpy
from rclpy.node import Node

from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu


class SimImu(Node):
    def __init__(self):
        super().__init__("sim_imu")

        self.quaternion_publisher = self.create_publisher(QuaternionStamped, "/filter/quaternion", 10)

        self.create_subscription(Imu, "gz/imu", self.on_imu, 10)

    def on_imu(self, msg: Imu):
        out = QuaternionStamped()
        out.header = msg.header
        out.quaternion = msg.orientation
        self.quaternion_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    sim_imu_node = SimImu()
    rclpy.spin(sim_imu_node)
    sim_imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()