from enum import Enum


class Topics(Enum):
    SENSORS = "sensors"  # TODO: Deprecate in favor of individual messages
    DVL = "/gz/dvl"  # TODO: Rename
    IMU_TRANSFORMED = "imu_transformed"
    SENSOR_DATA = "sensor/data"
    DVL_TWIST_SYNC = "/dvl/twist_sync"


class Nodes(Enum):
    PUBLISHER = "my_publisher"
    SUBSCRIBER = "my_subscriber"
    CONTROLLER = "robot_controller"
    SENSOR = "sensor_reader"


# Publishers (Node -> Topic)
PUBLISHERS = {
    Nodes.PUBLISHER: [Topics.MY_TOPIC],
    Nodes.CONTROLLER: [Topics.CMD_VEL],
    Nodes.SENSOR: [Topics.SENSOR_DATA],
}

# Subscribers (Node -> Topic)
SUBSCRIBERS = {
    Nodes.SUBSCRIBER: [Topics.MY_TOPIC],
    Nodes.CONTROLLER: [Topics.SENSOR_DATA],
}
