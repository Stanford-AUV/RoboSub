import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

import yaml, os

class PathLoader(Node):
    def __init__(self, yaml_path):
        super().__init__("path_loader")
        self.yaml_path = yaml_path

        self.waypoints_publisher = self.create_publisher(
            Path, "/waypoints", 10
        )

        self.path_index = 0
        self.segment_index = -1
        self.path: Path | None = None
        self.cycle_seconds = 10.0

        self.segments = self.read_yaml_path(self.yaml_path)
        self.advance_timer = self.create_timer(self.cycle_seconds, self.advance_waypoint)
        self.advance_waypoint()

    def read_yaml_path(self, yaml_path):
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        segments = []
        for segment in data:
            segments.append(data.get(segment))
        return segments

    def advance_waypoint(self):
        if self.segment_index == -1 or self.path_index == len(self.segments[self.segment_index]):
            if self.segment_index == len(self.segments) - 1:
                self.get_logger().info("All path segments have been streamed.")
                return
            # we want to load in the waypoint data
            self.segment_index += 1
            self.path_index = 0
            self.path = self.get_waypoint_data(self.segments[self.segment_index])
            self.waypoints_publisher.publish(self.path)
            self.get_logger().info(
                f"Published segment {self.segment_index}; advancing every {self.cycle_seconds:.0f}s."
            )
        
        if self.path is not None and self.path_index < len(self.path.poses):
            self.path_index += 1
            self.get_logger().info(
                f"Advancing to waypoint {self.path_index} in segment {self.segment_index}"
            )
    
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