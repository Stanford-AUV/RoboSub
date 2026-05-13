import rclpy
from rclpy.node import Node
from generic_sensor import GenericSensor
from msgs.msg import SensorsStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

_BIG = 1e9

class DepthSensor(GenericSensor):
    def __init__(self):
        super().__init__("depth", "depth_0")

        self.create_subscription(SensorsStamped, "/arduino/sensors", self.get_depth, 10)
        self.sensor_publishers = {
            "position": self.create_publisher(PoseWithCovarianceStamped, "/position", 10),
        }
        self.current_data = None


    def get_depth(self, msg):
        self.current_data = self._safe(msg.depth)
        self.publish_sensor_data()
    
    def _build_pose_cov(self):
        cov = np.zeros(36)
        pos_axes = self.get_axes("position")
        pos_cov = self.get_covariance("position")

        for i in range(3):
            cov[i * 6 + i] = _BIG

        if pos_cov is not None and pos_axes:
            for axis in pos_axes:
                idx = axis - 1
                cov[idx * 6 + idx] = float(pos_cov[idx][idx])

        return cov.tolist()
    
    def publish_sensor_data(self):
        if self.is_active("position"):
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.pose.position.z = self.current_data
            pose_msg.pose.pose.orientation.w = 1.0

            pose_msg.pose.covariance = self._build_pose_cov()

            self.sensor_publishers["position"].publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DepthSensor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()