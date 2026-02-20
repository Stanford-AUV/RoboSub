from rclpy.node import Node
import os
import yaml
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu


METADATA_FIELDS = {"covariance", "robot_pos", "robot_rot"}
DATA_TYPES = {"position", "rotation", "velocity", "angular", "accel"}


class GenericSensor(Node):
    """Base class for all sensor nodes.

    Reads sensors.yaml to determine which data types (pos, rot, vel, ang_vel, accel)
    are active and which axes each provides. Metadata fields (covariance, robot_pos,
    robot_rot) are stored as instance variables for subclass use.

    Subclasses are responsible for:
      - Creating publishers with the appropriate message types for their
        downstream consumers (e.g. sensor_msgs/Imu for robot_localization).
      - Implementing publish_sensor_data() to read hardware and publish.
      - Setting the correct frame_id (use a sensor-specific frame for raw data,
        or 'base_link' only after transforming to the body frame).
    """

    def __init__(self, node_name, sensor_name, yaml_path=None):
        super().__init__(node_name)

        if yaml_path is None:
            yaml_path = os.path.join(os.path.dirname(__file__), "..", "sensors.yaml")
        self.yaml_path = os.path.normpath(yaml_path)

        self.sensor_name = sensor_name
        self.active_axes = {}
        self._publishers = {
            "position": self.create_publisher(PoseWithCovarianceStamped, "/position", 10),
            "rotation": self.create_publisher(Imu, "/rotation", 10),
            "velocity": self.create_publisher(TwistWithCovarianceStamped, "/velocity", 10),
            "angular": self.create_publisher(Imu, "/angular", 10),
            "accel": self.create_publisher(Imu, "/accel", 10)
        }

        self.covariance = None
        self.robot_pos = None
        self.robot_rot = None

        self._load_config()

    def _load_sensors_yaml(self):
        if not os.path.exists(self.yaml_path):
            raise FileNotFoundError(f"Sensor config not found at {self.yaml_path}")
        with open(self.yaml_path, "r") as f:
            return yaml.safe_load(f)

    @staticmethod
    def _parse_axes(value):
        """Decode axis availability from the yaml value.

        0   -> []          (data type not available)
        3   -> [3]         (z only)
        123 -> [1, 2, 3]   (x, y, z)
        """
        if value is None or int(value) == 0:
            return []
        return [int(d) for d in str(int(value))]

    def _load_config(self):
        data = self._load_sensors_yaml()
        sensor_data = data.get(self.sensor_name)
        if sensor_data is None:
            self.get_logger().warning(
                f"No config for '{self.sensor_name}' in {self.yaml_path}"
            )
            return

        self.covariance = sensor_data.get("covariance")
        self.robot_pos = sensor_data.get("robot_pos")

        raw_rot = sensor_data.get("robot_rot")
        if raw_rot is not None:
            self.robot_rot = np.array(raw_rot, dtype=float)
        else:
            self.robot_rot = np.eye(3)

        for data_type in DATA_TYPES:
            raw_axes = sensor_data.get(data_type, 0)
            axes = self._parse_axes(raw_axes)
            if axes:
                self.active_axes[data_type] = axes

        self.get_logger().info(
            f"Sensor '{self.sensor_name}': "
            f"active data types = {list(self.active_axes.keys())}"
        )

    def is_active(self, data_type):
        """Return True if this sensor provides the given data type."""
        return data_type in self.active_axes

    def get_axes(self, data_type):
        """Return the list of active axis indices for a data type, e.g. [1, 2, 3]."""
        return self.active_axes.get(data_type, [])

    def publish_sensor_data(self):
        """Read from hardware and publish sensor data.

        Subclasses must override this. Typically invoked by a timer callback
        or in response to incoming hardware data.
        """
        raise NotImplementedError(
            f"{self.__class__.__name__} must implement publish_sensor_data()"
        )
