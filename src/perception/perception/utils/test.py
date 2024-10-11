# have: center (x, y) pixel in image space, position and orientation of robot in world space
# get: bounding rectangle in world space with closest object coordinate in bounding rectangle
#      (Extrapolated from center voxel (x, y, z) in world space)

# (x, y, z) for center
# (width, height, depth)
# orientation quaternion

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

CAM_MATRIX_LEFT = np.array([[500, 0, 50], [0, 400, 40], [0, 0, 1]])
BASELINE = 0.05

ROBOT_POS = np.array([5, 5, 5])
ROBOT_ORI = np.array([0.5, 0.5, 0.5])

# 2D position and orientation of the bounding box center
# yolov8_msgs/Pose2D center

# x, y, theta

OBJ_CENTER_X = 5
OBJ_CENTER_Y = 10
OBJ_CENTER_THETA = 3

# total size of the bounding box, in pixels, surrounding the object's center
# yolov8_msgs/Vector2 size

# size_x, size_y

BB_W = 500
BB_H = 400

# Stereo disparity map (you need to calculate this from stereo images)
# Replace this with your actual stereo matching implementation
DISP = 50  # Example value

# Calculate depth (z_world) using stereo disparity
z_world = (CAM_MATRIX_LEFT[0, 0] * BASELINE) / DISP

# Convert the center pixel to 3D camera coordinates
cam_center_x = (OBJ_CENTER_X - CAM_MATRIX_LEFT[0, 2]) * z_world / CAM_MATRIX_LEFT[0, 0]
cam_center_y = (OBJ_CENTER_Y - CAM_MATRIX_LEFT[1, 2]) * z_world / CAM_MATRIX_LEFT[1, 1]
cam_center_z = z_world

# Object center coordinates in the camera frame
obj_center_cam = np.array([cam_center_x, cam_center_y, cam_center_z])

# Get corners of bounding box
BB_TL = (0, 0)
BB_BL = (0, 100)
BB_TR = (100, 0)
BB_BR = (100, 100)

# Calculate 3D coordinates for the top-left and bottom-right corners of the bounding box
cam_top_left_x = (BB_TL[0] - CAM_MATRIX_LEFT[0, 2]) * z_world / CAM_MATRIX_LEFT[0, 0]
cam_top_left_y = (BB_TL[1] - CAM_MATRIX_LEFT[1, 2]) * z_world / CAM_MATRIX_LEFT[1, 1]
cam_bottom_right_x = (BB_BR[0] - CAM_MATRIX_LEFT[0, 2]) * z_world / CAM_MATRIX_LEFT[0, 0]
cam_bottom_right_y = (BB_BR[1] - CAM_MATRIX_LEFT[1, 2]) * z_world / CAM_MATRIX_LEFT[1, 1]

# Top-left and bottom-right coordinates in the camera frame
top_left_camera = np.array([cam_top_left_x, cam_top_left_y, z_world])
bottom_right_camera = np.array([cam_bottom_right_x, cam_bottom_right_y, z_world])

# Convert robot orientation to a rotation matrix
rotation_matrix = R.from_euler('xyz', ROBOT_ORI).as_matrix()

# Transform object coordinates from camera space to world space
object_center_world = rotation_matrix @ obj_center_cam + ROBOT_POS
top_left_world = rotation_matrix @ top_left_camera + ROBOT_POS
bottom_right_world = rotation_matrix @ bottom_right_camera + ROBOT_POS

# Calculate width, height of the bounding rectangle in world coordinates
bounding_box_width = np.linalg.norm(top_left_world - bottom_right_world, axis=0)[0]
bounding_box_height = np.linalg.norm(top_left_world - bottom_right_world, axis=0)[1]

# Compute bounding box corners in world space, assuming normal vector aligned with camera-object vector
object_direction = object_center_world - ROBOT_POS
object_direction_normalized = object_direction / np.linalg.norm(object_direction)

# Bounding rectangle is assumed to be in a plane perpendicular to this direction
# Construct the other two corners of the rectangle (perpendicular to the object direction)
right_vector = np.cross(object_direction_normalized, [0, 0, 1])  # Cross product to get a perpendicular vector
right_vector_normalized = right_vector / np.linalg.norm(right_vector)

# Adjust size for right and up vectors
up_vector = np.cross(right_vector_normalized, object_direction_normalized)
up_vector_normalized = up_vector / np.linalg.norm(up_vector)

# Calculate world coordinates for the four corners of the bounding rectangle
corner1_world = object_center_world - right_vector_normalized * bounding_box_width / 2 - up_vector_normalized * bounding_box_height / 2
corner2_world = object_center_world + right_vector_normalized * bounding_box_width / 2 - up_vector_normalized * bounding_box_height / 2
corner3_world = object_center_world + right_vector_normalized * bounding_box_width / 2 + up_vector_normalized * bounding_box_height / 2
corner4_world = object_center_world - right_vector_normalized * bounding_box_width / 2 + up_vector_normalized * bounding_box_height / 2

# Calculate the center point in world space (already done with object_center_world)
print(f"Object center in world space: {object_center_world}")

# To calculate orientation quaternion of the bounding box
# We need to build a rotation matrix from the right_vector and up_vector, forming the bounding box's orientation.
bounding_box_rotation_matrix = np.column_stack((right_vector_normalized, up_vector_normalized, object_direction_normalized))

# Convert the bounding box's rotation matrix to a quaternion
bounding_box_orientation_quaternion = R.from_matrix(bounding_box_rotation_matrix).as_quat()

print(f"Bounding box orientation (quaternion): {bounding_box_orientation_quaternion}")

print(f"Corner 1: {corner1_world}")
print(f"Corner 2: {corner2_world}")
print(f"Corner 3: {corner3_world}")
print(f"Corner 4: {corner4_world}")
