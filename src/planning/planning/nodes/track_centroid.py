import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray

class CentroidTracker(Node):
    def __init__(self):
        super().__init__("centroid_tracker")
        # Publisher for centroid waypoint
        self.waypoint_publisher = self.create_publisher(Odometry, "/waypoint", 10)
        # Subscriber to world-frame detections
        self.det_data_sub = self.create_subscription(
            Detection3DArray,
            "/detections_world",
            self.detection_callback,
            10
        )
    
    def detection_callback(self, msg: Detection3DArray):
        if not msg.detections:
            self.get_logger().info("No detections received.")
            return

        x_sum = 0.0
        y_sum = 0.0
        z_sum = 0.0
        count = 0

        self.get_logger().info(f"Processing {len(msg.detections)} detections.")

        for detection in msg.detections:
            res = detection.results[0]
            pos = res.pose.pose.position
            self.get_logger().info(f"Got position: x={pos.x}, y={pos.y}, z={pos.z}")

            x_sum += pos.x
            y_sum += pos.y
            z_sum += pos.z
            count += 1

        if count < 1:
            self.get_logger().info("No valid results to compute centroid.")
            return

        # Compute 3D centroid
        centroid_x = x_sum / count
        centroid_y = y_sum / count
        centroid_z = z_sum / count

        # Create Odometry message
        waypoint_msg = Odometry()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.pose.pose.position.x = centroid_x
        waypoint_msg.pose.pose.position.y = centroid_y
        waypoint_msg.pose.pose.position.z = centroid_z

        self.get_logger().info(f"Publishing 3D centroid at ({centroid_x:.2f}, {centroid_y:.2f}, {centroid_z:.2f})")
        self.waypoint_publisher.publish(waypoint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CentroidTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()