import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from scipy.spatial.transform import Rotation
import math

import yaml, os

class PathLoader(Node):
    def __init__(self, yaml_path):
        super().__init__("path_loader")
        self.yaml_path = yaml_path

        self.waypoints_publisher = self.create_publisher(
            Path, "/waypoints", 10
        )

        self.pose_subscription = self.create_subscription(
            Odometry, "/world/pose", self.update_tracker, 10
        )

        self.path_index = 0
        self.segment_index = -1
        self.path: Path | None = None
        self.waypoint_margin = 1.0  # meters - distance threshold to consider waypoint reached
        self.orientation_margin = 15.0  # degrees - angular threshold to consider orientation reached

        self.segments = self.read_yaml_path(self.yaml_path)

    def read_yaml_path(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        segments = []
        for segment in data:
            segments.append(data.get(segment))
        return segments

    def update_tracker(self, msg):
        if self.segment_index == -1 or self.path_index == len(self.segments[self.segment_index]):
            if self.segment_index == len(self.segments) - 1:
                return
            # we want to load in the waypoint data
            self.segment_index += 1
            self.path_index = 0
            self.path = self.get_waypoint_data(self.segments[self.segment_index])
            self.waypoints_publisher.publish(self.path)
        
        if self.path is not None and self.path_index < len(self.path.poses):
            current_pose = msg.pose.pose
            waypoint_pose = self.path.poses[self.path_index].pose
            
            distance, angle_diff_deg = self.get_distance_between_points(current_pose, waypoint_pose)
            
            if distance <= self.waypoint_margin and angle_diff_deg <= self.orientation_margin:
                self.path_index += 1
                self.get_logger().info(f"Reached waypoint {self.path_index - 1}, moving to waypoint {self.path_index}")
        
    
    def get_waypoint_data(self, segment):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for waypoint in segment["waypoints"]:

            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            pose_stamped.pose.position.x = waypoint["position"]["x"]
            pose_stamped.pose.position.y = waypoint["position"]["y"]
            pose_stamped.pose.position.z = waypoint["position"]["z"]

            roll = waypoint["orientation"]["roll"]
            pitch = waypoint["orientation"]["pitch"]
            yaw = waypoint["orientation"]["yaw"]
            orientation_quat = Rotation.from_euler(
                "xyz", [roll, pitch, yaw], degrees=True
            ).as_quat()
            pose_stamped.pose.orientation.x = orientation_quat[0]
            pose_stamped.pose.orientation.y = orientation_quat[1]
            pose_stamped.pose.orientation.z = orientation_quat[2]
            pose_stamped.pose.orientation.w = orientation_quat[3]

            path_msg.poses.append(pose_stamped)

        self.get_logger().info(f"Got waypoints for segment_{self.segment_index}.")
        return path_msg

    @staticmethod
    def get_distance_between_points(current_pose, waypoint_pose):
        # Calculate Euclidean distance
        dx = current_pose.position.x - waypoint_pose.position.x
        dy = current_pose.position.y - waypoint_pose.position.y
        dz = current_pose.position.z - waypoint_pose.position.z
        distance = (dx**2 + dy**2 + dz**2)**0.5
        
        current_quat = [
            current_pose.orientation.x,
            current_pose.orientation.y,
            current_pose.orientation.z,
            current_pose.orientation.w
        ]
        waypoint_quat = [
            waypoint_pose.orientation.x,
            waypoint_pose.orientation.y,
            waypoint_pose.orientation.z,
            waypoint_pose.orientation.w
        ]
        
        current_rot = Rotation.from_quat(current_quat)
        waypoint_rot = Rotation.from_quat(waypoint_quat)
        
        relative_rot = waypoint_rot * current_rot.inv()
        angle_diff_rad = relative_rot.magnitude()
        angle_diff_deg = abs(math.degrees(angle_diff_rad))
        
        return distance, angle_diff_deg

def main(args=None):
    SCRIPT_DIR = os.path.dirname(os.path.realpath(__file__))
    yaml_path = os.path.join(SCRIPT_DIR, "..", "sample_path.yaml")
    yaml_path = os.path.abspath(yaml_path)

    rclpy.init(args=args)
    node = PathLoader(yaml_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()