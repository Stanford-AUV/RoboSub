#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

class LocalizationTestNode(Node):
    def __init__(self):
        super().__init__('localization_test_node')
        # Publisher to simulate constant acceleration from the IMU node
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        # Subscriber to the EKF-filtered odometry output from ekf_filter_node
        self.odom_sub = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # List to track x positions over time
        self.x_positions = []

        # Publish rate and test duration settings
        self.publish_period = 0.1  # seconds (10 Hz)
        self.test_duration = 5.0   # seconds

        # Start timers for publishing and checking progress
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.create_timer(self.publish_period, self.publish_imu)
        self.create_timer(0.5, self.check_progress)

    def publish_imu(self):
        # Create an IMU message with a constant acceleration in x-direction
        imu_msg = Imu()
        now = self.get_clock().now().to_msg()
        imu_msg.header.stamp = now

        # Set a constant linear acceleration along x (e.g. 1 m/s^2)
        imu_msg.linear_acceleration.x = 0.0
        imu_msg.linear_acceleration.y = 1.0
        imu_msg.linear_acceleration.z = 0.0

        # Angular velocities are zero for this test
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0

        self.imu_pub.publish(imu_msg)
        self.get_logger().debug("Published IMU message with constant acceleration.")

    def odom_callback(self, msg):
        # Extract the x position from the odometry message
        x = msg.pose.pose.position.x
        self.x_positions.append(x)
        self.get_logger().info(f"Received EKF odometry x-position: {x:.3f}")

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
