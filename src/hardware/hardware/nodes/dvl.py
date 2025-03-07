import rclpy
from rclpy.node import Node
from msgs.msg import DVLData, DVLBeam, DVLTarget, DVLVelocity
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

from hardware.utils.dvl_utils.system import OutputData
from hardware.utils.dvl_utils.dvl_connect import Dvl
import numpy as np
import glob
import serial


class DVL(Node):
    def __init__(self, baudrate=115200):
        super().__init__("dvl_ros_bridge")

        # ROS publisher
        self.publisher = self.create_publisher(DVLData, "dvl", 10)

        # Connect to DVL
        self.dvl = self.autodetect_dvl_port(baudrate)
        if not self.dvl:
            self.get_logger().error("Failed to connect to DVL.")
            return

        # Register the callback function to process DVL data
        self.dvl.register_ondata_callback(self.update_data)

        # Start pinging
        if not self.dvl.exit_command_mode():
            self.get_logger().error("Failed to start pinging")

    def autodetect_dvl_port(self, baudrate, timeout=2):
        """Scan available serial ports and attempt to connect to the DVL."""
        possible_ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        for port in possible_ports:
            try:
                # Attempt a basic serial connection to check if the port is valid
                with serial.Serial(port, baudrate, timeout=timeout) as ser:
                    if not ser.is_open:
                        continue

                dvl = Dvl(port, baudrate)
                self.get_logger().info(f"Port: {port}")
                if dvl.is_connected():
                    return dvl

            except serial.SerialException:
                self.get_logger().debug(f"Port {port} is not available or is busy.")
            except serial.SerialTimeoutException:
                self.get_logger().debug(f"Port {port} timed out.")
            except Exception as e:
                self.get_logger().error(f"Unexpected error on port {port}: {e}")
        return None  # No valid port found

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

        # {'Count': 0,
        #  'Date': '2025/02/06',
        #  'Time': '20:33:42.380',
        #  'Coordinate': 'XYZ',
        #  'Velocity X': nan,
        #  'Velocity Y': nan,
        #  'Velocity Z': nan,
        #  'Velocity Err': nan,
        #  'Beam 1 range': nan,
        #  'Beam 2 range': nan,
        #  'Beam 3 range': nan,
        #  'Beam 4 range': nan,
        #  'Mean range': nan,
        #  'Speed of sound': 1500.0,
        #  'Input voltage': 12.010638236999512,
        #  'Transmit voltage': 35.282958984375,
        #  'Current': 0.35809072852134705,
        #  'Status': 255,
        #  'BIT count': 1,
        #  'BIT codes': 236
        # }

        # Construct DVLData message
        dvl_msg = DVLData()
        dvl_msg.header = Header()
        dvl_msg.header.stamp = timestamp

        # STUPID
        self.publisher.publish(dvl_msg)
        self.get_logger().info(f"Published DVL data: {str(dvl_msg)}")

        return

        T = np.array(
            [
                [-np.sqrt(2) / 2, np.sqrt(2) / 2, 0],
                [-np.sqrt(2) / 2, -np.sqrt(2) / 2, 0],
                [0, 0, 1],
            ]
        )

        # Set velocity information
        dvl_velocity = DVLVelocity()
        dvl_velocity.reference = 0  # Unknown reference frame
        x = data_dict["Velocity X"]
        self.get_logger().info(f"x {x}")
        if np.isnan(x):
            self.get_logger().error("Velocity X is nan")
            x = 0.0
        y = data_dict["Velocity Y"]
        if np.isnan(y):
            self.get_logger().error("Velocity Y is nan")
            y = 0.0
        z = data_dict["Velocity Z"]
        if np.isnan(z):
            self.get_logger().error("Velocity Z is nan")
            z = 0.0
        vel = T @ np.array([x, y, z])
        dvl_velocity.mean = Vector3()
        dvl_velocity.mean.x = vel[0]
        dvl_velocity.mean.y = vel[1]
        dvl_velocity.mean.z = vel[2]
        err = data_dict["Velocity Err"]
        if np.isnan(err):
            self.get_logger().error("Velocity Err is nan")
            err = 0.05
        covariance = np.eye(3, dtype=np.float32) * (err**2)
        dvl_velocity.covariance = covariance.flatten()

        # Set DVL target
        dvl_target = DVLTarget()
        dvl_target.type = 0  # Unknown target type
        r = data_dict["Mean range"]
        if np.isnan(r):
            self.get_logger().error("Mean range is nan")
            r = 0.0
        dvl_target.range_mean = r

        # Set beam data
        beam_ids = [1, 2, 3, 4]
        dvl_msg.beams = []
        for beam_id in beam_ids:
            beam = DVLBeam()
            beam.id = beam_id
            br = data_dict[f"Beam {beam_id} range"]
            if np.isnan(br):
                self.get_logger().error(f"Beam {beam_id} range is nan")
                br = 0.0
            beam.range_mean = br
            beam.locked = True  # Assume locked unless otherwise known

            # Beam velocity (not explicitly given in your data)
            beam.velocity = DVLVelocity()
            beam.velocity.reference = 0  # Unknown reference
            err = data_dict["Velocity Err"]
            if np.isnan(err):
                self.get_logger().error("Velocity Err is nan")
                err = 0.05
            if beam_id == 1:
                vx = data_dict["Velocity X"]
                if np.isnan(vx):
                    self.get_logger().error("Velocity X is nan")
                    vx = 0.0
                beam.velocity.mean = Vector3(x=vx, y=0.0, z=0.0)
            if beam_id == 2:
                vy = data_dict["Velocity Y"]
                if np.isnan(vy):
                    self.get_logger().error("Velocity Y is nan")
                    vy = 0.0
                beam.velocity.mean = Vector3(x=0.0, y=vy, z=0.0)
            if beam_id == 3:
                vz = data_dict["Velocity Z"]
                if np.isnan(vz):
                    self.get_logger().error("Velocity Z is nan")
                    vz = 0.0
                beam.velocity.mean = Vector3(x=0.0, y=0.0, z=vz)
            else:
                beam.velocity.mean = Vector3(x=err, y=err, z=err)
            covariance = np.eye(3) * (err**2)
            beam.velocity.covariance = covariance.flatten()

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
    node = DVL()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
