import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from msgs.msg import ThrustsStamped


class Thrusters(Node):
    """This node converts individual thruster magnitudes and directions into Gazebo messages to control the thrusters."""

    def __init__(self):
        super().__init__("thrusters")

        self._thruster_ids = [f"thruster_{i}" for i in range(1, 9)]
        self._gazebo_thrust_pubs = {}
        self.create_subscription(
            ThrustsStamped,
            f"thrusts",
            self.thruster_callback,
            10,
        )
        for thruster in self._thruster_ids:
            gazebo_thrust_pub = self.create_publisher(Float64, f"gz/{thruster}", 10)
            self._gazebo_thrust_pubs[thruster] = gazebo_thrust_pub

    def thruster_callback(self, msg: ThrustsStamped):
        thrusts = msg.thrusts
        self.get_logger().info(f"Publishing thrusts to Gazebo: {thrusts}")
        for i, thruster in enumerate(self._thruster_ids):
            thrust_msg = Float64()
            thrust_msg.data = 100 * thrusts[i]
            self._gazebo_thrust_pubs[thruster].publish(thrust_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Thrusters()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
