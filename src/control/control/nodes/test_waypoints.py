import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from msgs.msg import GeneratedPath  # Custom message import
from rclpy import Parameter
import random
import time
import json

from scipy.spatial.transform import Rotation


class WaypointTest(Node):
    def __init__(self):
        super().__init__("test_waypoints")

        self.declare_parameter("waypoints_topic", "path")
        self.declare_parameter("history_depth", Parameter.Type.INTEGER)

        history_depth = (
            self.get_parameter("history_depth").get_parameter_value().integer_value
        )

        self.waypoints_publisher = self.create_publisher(
            Path, "waypoints", history_depth
        )

        self.generated_path_subscriber = self.create_subscription(
            GeneratedPath, "generated_path", self.generated_path_callback, history_depth
        )  # Create test waypoints node

        self.get_logger().info("WaypointTest node has been started.")

        self.timer = self.create_timer(5, self.publish)

        self.get_logger().info("WaypointTest node has been tested.")

    def publish(self):
        # self.publish_random_waypoints()
        # Uncomment below to read from JSON file
        self.publish_waypoints_from_json(
            "/workspaces/RoboSub/src/planning/planning/nodes/test_waypoints.json"
        )

    def publish_random_waypoints(self):
        # Create a GivenPath message
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Generate random PoseStamped and Twist objects
        num_waypoints = 5  # Number of waypoints to generate
        for _ in range(num_waypoints):
            # Create a random PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()

            # Random position
            pose_stamped.pose.position.x = random.uniform(-10.0, 10.0)
            pose_stamped.pose.position.y = random.uniform(-10.0, 10.0)
            pose_stamped.pose.position.z = random.uniform(0.0, 5.0)

            # Random orientation (roll, pitch, yaw) in degrees
            roll = random.uniform(-180, 180)
            pitch = random.uniform(-180, 180)
            yaw = random.uniform(-180, 180)

            # Convert roll, pitch, yaw to quaternion
            orientation_quat = Rotation.from_euler(
                "xyz", [roll, pitch, yaw], degrees=True
            ).as_quat()
            pose_stamped.pose.orientation.x = orientation_quat[0]
            pose_stamped.pose.orientation.y = orientation_quat[1]
            pose_stamped.pose.orientation.z = orientation_quat[2]
            pose_stamped.pose.orientation.w = orientation_quat[3]

            # Append the generated pose_stamped to your list of waypoints
            path_msg.poses.append(pose_stamped)
            # path_msg.twists.append(twist)

        # Publish the GivenPath message
        self.waypoints_publisher.publish(path_msg)
        self.get_logger().info(f"Published {num_waypoints} random waypoints.")

    def generated_path_callback(self, msg: GeneratedPath):
        self.get_logger().info("Received generated path! Here it is:")
        self.get_logger().info(f"{msg}")

    def publish_waypoints_from_json(self, json_file_path):
        try:
            with open(json_file_path, "r") as f:
                data = json.load(f)

            # Create a Path message
            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            # Loop through waypoints in the JSON file
            for waypoint in data["waypoints"]:
                # Create PoseStamped for position and orientation
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "map"
                pose_stamped.header.stamp = self.get_clock().now().to_msg()

                # Set position
                pose_stamped.pose.position.x = waypoint["position"]["x"]
                pose_stamped.pose.position.y = waypoint["position"]["y"]
                pose_stamped.pose.position.z = waypoint["position"]["z"]

                # Set orientation from roll, pitch, yaw in degrees, converting to quaternion
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

                # Append pose to the path message
                path_msg.poses.append(pose_stamped)

            # Publish the path message
            self.waypoints_publisher.publish(path_msg)
            self.get_logger().info("Published waypoints from JSON file.")

        except FileNotFoundError:
            self.get_logger().error(f"File not found: {json_file_path}")
        except KeyError as e:
            self.get_logger().error(f"Missing key in JSON file: {e}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Error decoding JSON file: {json_file_path}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
