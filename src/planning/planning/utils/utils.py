import math
from scipy.spatial.transform import Rotation

def direction_to_quaternion(direction):
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

def generate_fibonacci_sphere_points(num_points: int):
    """
    Generate points on a sphere using a more uniform distribution to cover the whole sphere.
    
    Args:
        num_points (int): Number of points to generate.
    
    Returns:
        List of tuples representing directions as (x, y, z).
    """
    points = []
    phi = math.pi * (3 - math.sqrt(5))  # Golden angle in radians, this helps spread the points

    for i in range(num_points):
        # Uniformly distribute points on the sphere
        y = 1 - (i / float(num_points - 1)) * 2  # y goes from 1 to -1
        radius = math.sqrt(1 - y * y)  # Radius at y

        # Apply the golden angle to theta
        theta = phi * i  # Golden angle

        # Coordinates
        x = math.cos(theta) * radius
        z = math.sin(theta) * radius

        points.append((x, y, z))
    
    return points

