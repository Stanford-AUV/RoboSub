import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection2DArray

class CentroidTracker(Node):
    def __init__(self):
        super().__init__("centroid_tracker")
        self.waypoint_publisher = self.create_publisher(Odometry, "waypoint", 10)
        self.det_data_sub = self.create_subscription(Detection2DArray, "detection_data", self.detection_callback, 10)
    
    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            self.get_logger().info("No detections received.")
            return
        
        # Calculate centroid of all detected objects
        x_sum = 0.0
        y_sum = 0.0
        count = len(msg.detections)
        
        for detection in msg.detections:
            if detection.hypothesis.class_id == "" or detection.bbox.size_x == 0 or detection.bbox.size_y == 0:
                continue
            x_sum += detection.bbox.center.position.x
            y_sum += detection.bbox.center.position.y

        centroid_x = x_sum / count
        centroid_y = y_sum / count
        
        # Create and publish Odometry message for the centroid
        waypoint_msg = Odometry()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = "map"
        waypoint_msg.pose.pose.position.x = centroid_x
        waypoint_msg.pose.pose.position.y = centroid_y
        waypoint_msg.pose.pose.position.z = 0.0  # Assuming 2D plane
        
        self.get_logger().info(f"Publishing centroid waypoint at ({centroid_x}, {centroid_y})")
        self.waypoint_publisher.publish(waypoint_msg)