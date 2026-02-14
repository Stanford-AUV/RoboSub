from rclpy.node import Node
import os
import yaml

class GenericSensor(Node):
    def __init__(self, name, path):
        self.name = name
        self.path = path
        self.publishers = {}
        self.build_publishers()

    def load_sensors_yaml(self):
        if not os.path.exists(self.path):
            raise FileNotFoundError("Noooooo! No yaml path exists :(. Get a new sensor.")

        with open(self.path, "r") as f:
            data = yaml.safe_load(f)

        return data

    def build_publishers(self):
        data = self.load_sensors_yaml()
        if not (data.get(self.name)):
            return
        sensor_data = data[self.name]
        
        for key, val in sensor_data:
            val = int(val)
            if val != 0:
                self.publishers[key] = self.create_publisher(float[3], f"/{self.name}/{self.key}", 10)

    def publish_sensor_data(self):
        raise NotImplementedError("Implement this (publish_sensor_data)!")
    