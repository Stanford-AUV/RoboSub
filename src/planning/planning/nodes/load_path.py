import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import sys
import yaml
import os
from rclpy.utilities import remove_ros_args


class PathLoader(Node):
    def __init__(self, yaml_path):
        super().__init__("path_loader")
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.waypoints_publisher = self.create_publisher(
            Path, "/waypoints", latched_qos
        )
        segments = self.read_yaml_path(yaml_path)
        path_msg = self.build_path(segments)
        self.waypoints_publisher.publish(path_msg)
        self.get_logger().info(
            f"Published {len(path_msg.poses)} waypoints from {len(segments)} segment(s)."
        )

    def read_yaml_path(self, yaml_path):
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)
        return [data[key] for key in data]

    def build_path(self, segments):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for segment in segments:
            for waypoint in segment["waypoints"]:
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.header.stamp = path_msg.header.stamp
                pose_stamped.pose.position.x = waypoint["position"]["x"]
                pose_stamped.pose.position.y = waypoint["position"]["y"]
                pose_stamped.pose.position.z = waypoint["position"]["z"]
                roll = waypoint["orientation"]["roll"]
                pitch = waypoint["orientation"]["pitch"]
                yaw = waypoint["orientation"]["yaw"]
                q = Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_quat()
                pose_stamped.pose.orientation.x = q[0]
                pose_stamped.pose.orientation.y = q[1]
                pose_stamped.pose.orientation.z = q[2]
                pose_stamped.pose.orientation.w = q[3]
                path_msg.poses.append(pose_stamped)
        return path_msg


def main(args=None):
    script_dir = os.path.dirname(os.path.realpath(__file__))
    default_yaml = os.path.abspath(os.path.join(script_dir, "..", "prequal.yaml"))
    argv = remove_ros_args(args=sys.argv)
    if len(argv) > 1:
        yaml_path = os.path.abspath(os.path.expanduser(argv[1]))
    else:
        yaml_path = default_yaml
    if not os.path.isfile(yaml_path):
        print(f"Error: YAML file not found: {yaml_path}", file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=args)
    node = PathLoader(yaml_path)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
