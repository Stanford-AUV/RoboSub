#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from msgs.msg import DVLData, Float32Stamped  # Ensure this message type is available in your workspace
import math

# Hyperparameter: Set to True for testing mode (publishing sensor data and checking progress),
# or False for monitoring mode (only listening to odometry).
TEST_MODE = False

class LocalizationTestNode(Node):
    def __init__(self):
        super().__init__('localization_test_node')

        # Always subscribe to EKF-filtered odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )
        self.x_positions = []  # List to track x positions over time

        if TEST_MODE:
            # Publishers to simulate sensor outputs:
            # IMU: for acceleration measurements
            self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
            # DVL: for velocity measurements; using custom DVLData message type
            self.dvl_pub = self.create_publisher(DVLData, 'dvl', 10)
            # Depth: for depth measurements; using standard Float32 message
            self.depth_pub = self.create_publisher(Float32Stamped, 'depth', 10)

            # Publish rate and test duration settings
            self.publish_period = 1/600  # seconds (10 Hz)
            self.test_duration = 5.0   # seconds

            # Start timers for publishing sensor data and checking progress
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.create_timer(self.publish_period, self.publish_sensors)
            self.create_timer(0.5, self.check_progress)
            self.get_logger().info("Running in TEST_MODE: Publishing sensor data.")
        else:
            self.get_logger().info("Running in MONITORING mode: Only subscribing to odometry.")

    def publish_sensors(self):
        now = self.get_clock().now().to_msg()

        # --- Publish IMU message ---
        imu_msg = Imu()
        imu_msg.header.stamp = now
        # For testing, set constant acceleration in x (or any axis as needed)
        imu_msg.linear_acceleration.x = 1.0  # 1 m/s^2 in x-direction
        imu_msg.linear_acceleration.y = 0.0
        imu_msg.linear_acceleration.z = 0.0
        # Zero angular velocity for simplicity:
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        self.imu_pub.publish(imu_msg)
        self.get_logger().debug("Published IMU message with acceleration in x.")

        # --- Publish DVL message ---
        dvl_msg = DVLData()
        # Assuming DVLData contains a field "velocity" with a sub-field "mean" (vector)
        dvl_msg.velocity.mean.x = 0.2  # 0.2 m/s in x-direction
        dvl_msg.velocity.mean.y = 0.0
        dvl_msg.velocity.mean.z = 0.0
        self.dvl_pub.publish(dvl_msg)
        self.get_logger().debug("Published DVL message with constant velocity.")

        # --- Publish Depth message ---
        depth_msg = Float32Stamped()
        depth_msg.data = 10.0  # Fixed depth, e.g., 10 meters
        self.depth_pub.publish(depth_msg)
        self.get_logger().debug("Published Depth message with constant depth.")

    def odom_callback(self, msg):
        # Extract the full position from the odometry message
        pos = msg.pose.pose.position
        self.x_positions.append(pos.x)
        self.get_logger().info(
            f"Received EKF odometry position: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}"
        )

    def check_progress(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        if len(self.x_positions) >= 2:
            # Compare the first and latest x positions
            if self.x_positions[-1] > self.x_positions[0]:
                self.get_logger().info("Test progress: x position is increasing.")
            else:
                self.get_logger().error("Test failure: x position is not increasing.")

        # End test after the test duration has elapsed
        if elapsed > self.test_duration:
            if len(self.x_positions) >= 2 and self.x_positions[-1] > self.x_positions[0]:
                self.get_logger().info("Test passed: Position increased over time.")
            else:
                self.get_logger().error("Test failed: Position did not increase as expected.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    test_node = LocalizationTestNode()
    try:
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        pass
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
