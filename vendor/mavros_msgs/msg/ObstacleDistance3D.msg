# Obstacle Distance 3D message
# Represents a single obstacle position in 3D space
# 
# https://mavlink.io/en/messages/ardupilotmega.html#OBSTACLE_DISTANCE_3D
# MAVLink Message ID: 11037

std_msgs/Header header

# MAV_DISTANCE_SENSOR enum
uint8 sensor_type

# MAV_FRAME enum  
uint8 frame

# Unique ID for the obstacle
uint16 obstacle_id

# Obstacle position in meters
geometry_msgs/Point position

# Sensor range in meters
float32 min_distance
float32 max_distance
