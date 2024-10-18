# have: center (x, y) pixel in image space, position and orientation of robot in world space
# get: bounding rectangle in world space witDEPTH_MEASUh closest object coordinate in bounding rectangle
#      (Extrapolated from center voxel (x, y, z) in world space)

# (x, y, z) for center
# (width, height, depth)
# orientation quaternion

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

# --- Camera and Environment Constants ---
CAM_MATRIX_LEFT = np.array(
    [[500, 0, 50], [0, 400, 40], [0, 0, 1]]
)  # Example camera intrinsic matrix
DISTORTION_COEFFS = np.array(
    [0.1, -0.25, 0.0001, 0.0002, 0.05]
)  # Example distortion coefficients

BASELINE = 0.05  # Distance between left and right stereo cameras in meters
# MAX_DISPARITY = 128  # Maximum disparity for stereo matching (affects depth precision)

# Updated camera details
DEPTH_MEASURING_RANGE_MIN = 0.2  # Minimum depth range in meters
DEPTH_MEASURING_RANGE_MAX = 35.0  # Maximum depth range in meters

# RGB Camera specs (IMX378)
# RGB_CAMERA_RESOLUTION = (4032, 3040)  # 12MP resolution
# RGB_FOV_D = 81  # Diagonal FOV in degrees
# RGB_FOV_H = 69  # Horizontal FOV in degrees
# RGB_FOV_V = 55  # Vertical FOV in degrees
# RGB_MAX_FPS = 60  # Maximum FPS for RGB camera
# RGB_FOCAL_LENGTH = 4.81  # Focal length in mm
# RGB_APERTURE = 2.0  # Aperture
# RGB_DISTORTION = 0.01  # Distortion < 1%
# RGB_SHUTTER = "Rolling"  # Rolling shutter

# Binocular Camera (OV9282)
# DEPTH_CAMERA_RESOLUTION = (1280, 800)  # 1MP resolution
# DEPTH_FOV_D = 89  # Diagonal FOV in degrees
# DEPTH_FOV_H = 80  # Horizontal FOV in degrees
# DEPTH_FOV_V = 55  # Vertical FOV in degrees
# DEPTH_MAX_FPS = 120  # Maximum FPS for depth camera
# DEPTH_FOCAL_LENGTH = 2.35  # Focal length in mm
# DEPTH_APERTURE = 2.2  # Aperture
# DEPTH_DISTORTION = 0.015  # Distortion < 1.5%
# DEPTH_SHUTTER = "Global"  # Global shutter

# --- Underwater Constants ---
REFRACTIVE_INDEX_AIR = 1.0  # Refractive index of air
REFRACTIVE_INDEX_WATER = 1.33  # Refractive index of water
ATTENUATION_COEFF_RED = 0.3  # Light attenuation factors in water for RGB
ATTENUATION_COEFF_GREEN = 0.1
ATTENUATION_COEFF_BLUE = 0.05
TURBIDITY_FACTOR = 0.8  # Factor to account for water clarity (0 for clear, 1 for murky)

# --- Robot and Object Constraints ---
ROBOT_POS = np.array([5, 5, 5])  # Robot position in world space
ROBOT_ORI = np.array([0.5, 0.5, 0.5])  # Robot orientation (Euler angles)

# Submarine velocity (affects motion blur)
# SUBMARINE_VELOCITY = np.array([0.1, 0.05, 0])  # Example velocity in m/s

# Object detection information (from yolov8)
OBJ_CENTER_X = 5  # x pixel coordinate of object center
OBJ_CENTER_Y = 10  # y pixel coordinate of object center
OBJ_CENTER_THETA = 3  # Orientation of the object (theta in image space)

BB_W = 500  # Bounding box width in pixels
BB_H = 400  # Bounding box height in pixels

# Stereo disparity map
DISP = 50  # Disparity value (you should calculate this from stereo images)

# --- Depth and Refraction Calculations ---

# Calculate depth (z_world) using stereo disparity
z_world = (CAM_MATRIX_LEFT[0, 0] * BASELINE) / DISP

# Adjust z_world for underwater refraction
z_world = z_world * (REFRACTIVE_INDEX_WATER / REFRACTIVE_INDEX_AIR)

# Ensure z_world is within detectable range
if z_world < DEPTH_MEASURING_RANGE_MIN or z_world > DEPTH_MEASURING_RANGE_MAX:
    raise ValueError("Object is out of detection range.")

# --- Correct for Lens Distortion ---
# Correct the object center pixel using distortion coefficients
OBJ_CENTER_CORRECTED = cv2.undistortPoints(
    np.array([[OBJ_CENTER_X, OBJ_CENTER_Y]]), CAM_MATRIX_LEFT, DISTORTION_COEFFS
)

# Convert corrected center pixel to 3D camera coordinates
cam_center_x = (
    (OBJ_CENTER_CORRECTED[0][0][0] - CAM_MATRIX_LEFT[0, 2])
    * z_world
    / CAM_MATRIX_LEFT[0, 0]
)
cam_center_y = (
    (OBJ_CENTER_CORRECTED[0][0][1] - CAM_MATRIX_LEFT[1, 2])
    * z_world
    / CAM_MATRIX_LEFT[1, 1]
)
cam_center_z = z_world

# Object center coordinates in the camera frame
obj_center_cam = np.array([cam_center_x, cam_center_y, cam_center_z])

# --- Attenuation Due to Water ---
# Attenuate color channels based on distance and water turbidity
red_attenuation = np.exp(-ATTENUATION_COEFF_RED * z_world * TURBIDITY_FACTOR)
green_attenuation = np.exp(-ATTENUATION_COEFF_GREEN * z_world * TURBIDITY_FACTOR)
blue_attenuation = np.exp(-ATTENUATION_COEFF_BLUE * z_world * TURBIDITY_FACTOR)
attenuated_color = np.array(
    [red_attenuation, green_attenuation, blue_attenuation]
)  # For object appearance

# --- Calculate Corners of the Bounding Box in Camera Space ---
# Assuming a 2D bounding box around the object center in pixel space, we calculate 3D coordinates
BB_TL = (OBJ_CENTER_X - BB_W / 2, OBJ_CENTER_Y - BB_H / 2)
BB_BR = (OBJ_CENTER_X + BB_W / 2, OBJ_CENTER_Y + BB_H / 2)

cam_top_left_x = (BB_TL[0] - CAM_MATRIX_LEFT[0, 2]) * z_world / CAM_MATRIX_LEFT[0, 0]
cam_top_left_y = (BB_TL[1] - CAM_MATRIX_LEFT[1, 2]) * z_world / CAM_MATRIX_LEFT[1, 1]
cam_bottom_right_x = (
    (BB_BR[0] - CAM_MATRIX_LEFT[0, 2]) * z_world / CAM_MATRIX_LEFT[0, 0]
)
cam_bottom_right_y = (
    (BB_BR[1] - CAM_MATRIX_LEFT[1, 2]) * z_world / CAM_MATRIX_LEFT[1, 1]
)

# Convert corners to camera frame
top_left_camera = np.array([cam_top_left_x, cam_top_left_y, z_world])
bottom_right_camera = np.array([cam_bottom_right_x, cam_bottom_right_y, z_world])

# --- Transform from Camera Space to World Space ---
# Convert robot orientation to a rotation matrix
rotation_matrix = R.from_euler("xyz", ROBOT_ORI).as_matrix()

# Transform object coordinates from camera space to world space
object_center_world = rotation_matrix @ obj_center_cam + ROBOT_POS
top_left_world = rotation_matrix @ top_left_camera + ROBOT_POS
bottom_right_world = rotation_matrix @ bottom_right_camera + ROBOT_POS

# --- Calculate Bounding Box Dimensions ---
bounding_box_width = np.linalg.norm(top_left_world - bottom_right_world, axis=0)[0]
bounding_box_height = np.linalg.norm(top_left_world - bottom_right_world, axis=0)[1]

# Compute object direction from robot to object
object_direction = object_center_world - ROBOT_POS
object_direction_normalized = object_direction / np.linalg.norm(object_direction)

# --- Compute Bounding Rectangle Normal to Object Direction ---
right_vector = np.cross(object_direction_normalized, [0, 0, 1])
right_vector_normalized = right_vector / np.linalg.norm(right_vector)

up_vector = np.cross(right_vector_normalized, object_direction_normalized)
up_vector_normalized = up_vector / np.linalg.norm(up_vector)

# Calculate world coordinates for the four corners of the bounding rectangle
corner1_world = (
    object_center_world
    - right_vector_normalized * bounding_box_width / 2
    - up_vector_normalized * bounding_box_height / 2
)
corner2_world = (
    object_center_world
    + right_vector_normalized * bounding_box_width / 2
    - up_vector_normalized * bounding_box_height / 2
)
corner3_world = (
    object_center_world
    + right_vector_normalized * bounding_box_width / 2
    + up_vector_normalized * bounding_box_height / 2
)
corner4_world = (
    object_center_world
    - right_vector_normalized * bounding_box_width / 2
    + up_vector_normalized * bounding_box_height / 2
)

# --- Calculate Orientation Quaternion ---
bounding_box_rotation_matrix = np.column_stack(
    (right_vector_normalized, up_vector_normalized, object_direction_normalized)
)
bounding_box_orientation_quaternion = R.from_matrix(
    bounding_box_rotation_matrix
).as_quat()

# --- Print Results ---
print(f"Object center in world space: {object_center_world}")
print(
    f"Bounding box corners in world space: {corner1_world}, {corner2_world}, {corner3_world}, {corner4_world}"
)
print(f"Bounding box orientation (quaternion): {bounding_box_orientation_quaternion}")
print(f"Attenuated object color due to water: {attenuated_color}")
