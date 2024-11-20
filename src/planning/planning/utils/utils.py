import math
from scipy.spatial.transform import Rotation

def _direction_to_quaternion(self, direction):
    """
    Convert a direction vector to a quaternion.
    
    Args:
        direction (tuple): A direction vector (x, y, z).
    
    Returns:
        List representing the quaternion [x, y, z, w].
    """
    x, y, z = direction
    yaw = math.atan2(y, x)
    pitch = math.asin(-z)
    roll = 0  # Assuming no roll is required for the camera

    # Convert Euler angles (roll, pitch, yaw) to quaternion
    rotation = Rotation.from_euler('xyz', [roll, pitch, yaw])
    quaternion = rotation.as_quat()
    return quaternion

def _generate_fibonacci_sphere_points(self, num_points: int):
    """
    Generate points on a sphere using the Fibonacci sphere algorithm.
    
    Args:
        num_points (int): Number of points to generate.
    
    Returns:
        List of tuples representing directions as (x, y, z).
    """
    points = []
    phi = math.pi * (3 - math.sqrt(5))  # Golden angle in radians

    for i in range(num_points):
        y = 1 - (i / float(num_points - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # Radius at y

        theta = phi * i  # Angle around the sphere
        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points.append((x, y, z))
    return points

