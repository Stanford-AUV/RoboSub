import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistWithCovarianceStamped
from msgs.msg import DVLData

class SimDVL(Node):
    def __init__(self):
        super().__init__("sim_dvl")

        self.declare_parameter("velocity_variance", 0.01)
        self.velocity_variance = self.get_parameter("velocity_variance").value

        self.velocity_publisher = self.create_publisher(TwistWithCovarianceStamped, "/velocity", 10)
        self.create_subscription(DVLData, "gz/dvl", self.on_dvl,10)

    def on_dvl(self, msg: DVLData):

        locked_beams_count = sum(beam.locked for beam in msg.beams)

        if locked_beams_count < 3:
            return

        out = TwistWithCovarianceStamped()
        out.header = msg.header
        out.twist.twist.linear = msg.velocity.mean

        covariance = [0.0] * 36
        covariance[0] = covariance[7] = covariance[14] = self.velocity_variance
        covariance[21] = covariance[28] = covariance[35] = 1e9
        out.twist.covariance = covariance

        self.velocity_publisher.publish(out)



def main(args=None):
    rclpy.init(args=args)
    node = SimDVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()