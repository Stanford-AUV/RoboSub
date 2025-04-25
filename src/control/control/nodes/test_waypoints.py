import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from msgs.srv import GetWaypoints
from rclpy import Parameter
import random
import os
import json

from scipy.spatial.transform import Rotation


class WaypointTest(Node):
    def __init__(self):
        super().__init__("test_waypoints")
        self.get_logger().info("WaypointTest node has been started.")
        self.waypoints_srv = self.create_service(GetWaypoints, 'get_waypoints', self.get_waypoints)

    def get_waypoints(self, request: GetWaypoints.Request, response: GetWaypoints.Response):
        # waypoints = self.get_random_waypoints()
        # Uncomment below to read from JSON file
        waypoints = self.get_waypoints_from_json(
            os.path.join(os.path.dirname(__file__), "test_waypoints.json")
        )
        if waypoints is None:
            self.get_logger().error("Failed to get waypoints from JSON file.")
        else:
            response.waypoints = waypoints
        return response

    def get_random_waypoints(self):
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
        self.get_logger().info(f"Got {num_waypoints} random waypoints.")
        return path_msg

    def get_waypoints_from_json(self, json_file_path):
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
            self.get_logger().info("Got waypoints from JSON file.")
            return path_msg

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
