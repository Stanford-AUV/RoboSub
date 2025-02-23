import rclpy
from rclpy.node import Node
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from gz.transport13 import Node as GzNode

from gz.msgs10.dvl_velocity_tracking_pb2 import DVLVelocityTracking


class DVLBridgeNode(Node):
    def __init__(self):
        super().__init__("dvl_bridge")
        self.publisher = self.create_publisher(DVLData, "dvl", 10)
        self.gz_node = GzNode()
        self.gz_node.subscribe(DVLVelocityTracking, "dvl", self.dvl_callback)

    def dvl_callback(self, msg: DVLVelocityTracking):
        self.get_logger().info("Received DVL message")

        # Convert Gazebo DVL message to ROS 2 DVLData message
        ros_msg = DVLData()
        ros_msg.header = Header()
        ros_msg.header.stamp = self.get_clock().now().to_msg()
        ros_msg.type = msg.type

        # Convert target information
        ros_target = DVLTarget()
        ros_target.type = msg.target.type
        ros_target.range_mean = msg.target.range.mean
        ros_msg.target = ros_target

        # Convert velocity information
        ros_velocity = DVLVelocity()
        ros_velocity.reference = msg.velocity.reference
        ros_velocity.mean = Vector3(
            x=msg.velocity.mean.x,
            y=msg.velocity.mean.y,
            z=msg.velocity.mean.z,
        )
        ros_velocity.covariance = list(msg.velocity.covariance)
        ros_msg.velocity = ros_velocity

        # Convert each beam from Gazebo to ROS 2
        for beam in msg.beams:
            ros_beam = DVLBeam()
            ros_beam.id = beam.id
            ros_beam.range_mean = beam.range.mean
            ros_beam.locked = beam.locked

            # Convert the beam's velocity information
            ros_beam.velocity = DVLVelocity()
            ros_beam.velocity.reference = beam.velocity.reference
            ros_beam.velocity.mean = Vector3(
                x=beam.velocity.mean.x, y=beam.velocity.mean.y, z=beam.velocity.mean.z
            )
            ros_beam.velocity.covariance = list(beam.velocity.covariance)

            ros_msg.beams.append(ros_beam)

        # Publish the ROS 2 message
        self.publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DVLBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
