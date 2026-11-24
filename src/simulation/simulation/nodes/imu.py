import rclpy
from rclpy.node import Node

from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu

import math
import numpy as np
from scipy.spatial.transform import Rotation as R


class SimImu(Node):
    def __init__(self):
        super().__init__("sim_imu")
        
        self.declare_parameter("yaw_drift_deg_per_s", -0.07)
        self.drift_rate = self.get_parameter("yaw_drift_deg_per_s").value
        self.start_time = None

        # Publish List
        self.quaternion_publisher = self.create_publisher(QuaternionStamped, "/filter/quaternion", 10)
        
        # Suscribe List
        self.create_subscription(Imu, "gz/imu", self.on_imu, 10)

    def on_imu(self, msg: Imu):
        seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.start_time is None:
            self.start_time = seconds
        
        yaw_drift = math.radians(self.drift_rate) * (seconds - self.start_time)

        gz_true_orientation = R.from_quat([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        xsens_drifted_orientation = R.from_euler("z", yaw_drift) * gz_true_orientation
        xsens_noisy_drifted_orientation = xsens_drifted_orientation * R.from_euler("xyz", np.random.normal(0, math.radians(0.06), 3))
        x, y, z, w = xsens_noisy_drifted_orientation.as_quat()
        
        out = QuaternionStamped()
        out.header = msg.header
        out.quaternion.x = float(x)
        out.quaternion.y = float(y)
        out.quaternion.z = float(z)
        out.quaternion.w = float(w)
        self.quaternion_publisher.publish(out)

        # self.get_logger().info(f"drift {math.degrees(yaw_drift):.1f} deg", throttle_duration_sec=1.0)
        # noise_deg = math.degrees((xsens_drifted_orientation.inv() * xsens_noisy_drifted_orientation).magnitude())
        # self.get_logger().info(f"noise {noise_deg:.3f} deg", throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    sim_imu_node = SimImu()
    rclpy.spin(sim_imu_node)
    sim_imu_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()