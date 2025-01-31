import rclpy
from rclpy.node import Node
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

from hardware.utils.dvl.system import OutputData
from hardware.utils.dvl.dvl import Dvl
import numpy as np


class DVLROSBridge(Node):
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        super().__init__("dvl_ros_bridge")

        # ROS publisher
        self.publisher = self.create_publisher(DVLData, "/gz/dvl", 10)

        # Connect to DVL
        self.dvl = Dvl(port, baudrate)
        if not self.dvl.is_connected():
            self.get_logger().error("Failed to connect to DVL.")
            return

        # Register the callback function to process DVL data
        self.dvl.register_ondata_callback(self.update_data)

        # Start pinging
        if not self.dvl.exit_command_mode():
            self.get_logger().error("Failed to start pinging")

    def update_data(self, output_data: OutputData, obj):
        """Callback function to process and publish DVL data."""
        del obj
        if output_data is None:
            return

        # Extract timestamp
        timestamp = self.get_clock().now().to_msg()

        # Extract raw values from settings
        settings = output_data.get_settings()
        data_dict = {setting.name: setting.value for setting in settings}

        # Construct DVLData message
        dvl_msg = DVLData()
        dvl_msg.header = Header()
        dvl_msg.header.stamp = timestamp

        # Set velocity information
        dvl_velocity = DVLVelocity()
        dvl_velocity.reference = 0  # Unknown reference frame
        dvl_velocity.mean = Vector3(
            x=data_dict.get("Velocity X", 0.0),
            y=data_dict.get("Velocity Y", 0.0),
            z=data_dict.get("Velocity Z", 0.0),
        )
        dvl_velocity.covariance = np.eye(3).flatten().tolist()  # Covariance unknown

        # Set DVL target
        dvl_target = DVLTarget()
        dvl_target.type = 0  # Unknown target type
        dvl_target.range_mean = data_dict.get("Mean range", 0.0)

        # Set beam data
        beam_ids = [1, 2, 3, 4]
        dvl_msg.beams = []
        for beam_id in beam_ids:
            beam = DVLBeam()
            beam.id = beam_id
            beam.range_mean = data_dict.get(f"Beam {beam_id} range", 0.0)
            beam.locked = True  # Assume locked unless otherwise known

            # Beam velocity (not explicitly given in your data)
            beam.velocity = DVLVelocity()
            beam.velocity.reference = 0  # Unknown reference
            beam.velocity.mean = Vector3(x=0.0, y=0.0, z=0.0)
            beam.velocity.covariance = np.eye(3).flatten().tolist()

            dvl_msg.beams.append(beam)

        # Assign message values
        dvl_msg.type = 0  # Unknown type
        dvl_msg.velocity = dvl_velocity
        dvl_msg.target = dvl_target

        # Publish message
        self.publisher.publish(dvl_msg)
        self.get_logger().info(f"Published DVL data: {str(dvl_msg)}")


def main(args=None):
    rclpy.init(args=args)
    node = DVLROSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
