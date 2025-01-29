import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from utils.utils import *
from mission_planner import Mission

class TestMission(Mission):
    def __init__(self):
        # Initialize the parent Mission class
        super().__init__()

    def test_generate_and_publish_path(self):
        """
        Test the generation and publishing of the Fibonacci sphere look-around path.
        """
        overlap_percentage = 0.2  # Adjust the overlap percentage as needed
        self.look_around()  # Generate and publish the path
        # self.look_in_all_directions(overlap_percentage=overlap_percentage)  # Generate and publish the path


def main(args=None):
    rclpy.init(args=args)

    # Instantiate TestMission which extends the Mission class
    test_mission = TestMission()

    # Run the test method to generate and publish time path
    test_mission.test_generate_and_publish_path()

    # Spin the node to keep it active (for ROS to process callbacks)
    rclpy.spin(test_mission)

    # Clean up the node and shutdown when done
    test_mission.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
