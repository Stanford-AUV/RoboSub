#!/usr/bin/env python3

import depthai as dai
import cv2
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = True

# Air/water testing boolean
is_underwater = False  # Set to True for underwater testing

# --- Camera and Environment Constants ---
CAM_MATRIX_LEFT = np.array(
    [[500, 0, 50], [0, 400, 40], [0, 0, 1]], dtype=np.float32
)  # Example camera intrinsic matrix
DISTORTION_COEFFS = np.array(
    [0.1, -0.25, 0.0001, 0.0002, 0.05], dtype=np.float32
)  # Example distortion coefficients

BASELINE = 0.05  # Distance between left and right stereo cameras in meters

# Updated camera details
DEPTH_MEASURING_RANGE_MIN = 0.2  # Minimum depth range in meters
DEPTH_MEASURING_RANGE_MAX = 35.0  # Maximum depth range in meters

# --- Underwater Constants ---
REFRACTIVE_INDEX_AIR = 1.0  # Refractive index of air
REFRACTIVE_INDEX_WATER = 1.33  # Refractive index of water
ATTENUATION_COEFF_RED = 0.3  # Light attenuation factors in water for RGB
ATTENUATION_COEFF_GREEN = 0.1
ATTENUATION_COEFF_BLUE = 0.05
TURBIDITY_FACTOR = 0.8  # Factor to account for water clarity (0 for clear, 1 for murky)

# --- Robot and Object Constraints ---
ROBOT_POS = np.array([5, 5, 5], dtype=np.float32)  # Robot position in world space
ROBOT_ORI = np.array(
    [0.5, 0.5, 0.5], dtype=np.float32
)  # Robot orientation (Euler angles)

# Simulated object movement parameters
OBJ_CENTER_X = 50  # Initial x pixel coordinate of object center
OBJ_CENTER_Y = 100  # Initial y pixel coordinate of object center
move_direction = 1  # Direction to move the bounding box (1 for down, -1 for up)

# Stereo disparity map
DISP = 50  # Disparity value (you should calculate this from stereo images)

# --- Depth and Refraction Calculations ---


def calculate_z_world(disp):
    """Calculate depth using disparity."""
    return (CAM_MATRIX_LEFT[0, 0] * BASELINE) / disp


# --- Correct for Lens Distortion ---
def undistort_points(x, y):
    """Undistort object center pixel using distortion coefficients."""
    return cv2.undistortPoints(
        np.array([[x, y]], dtype=np.float32), CAM_MATRIX_LEFT, DISTORTION_COEFFS
    )


# --- Create pipeline ---
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
depth = pipeline.create(dai.node.StereoDepth)
xout = pipeline.create(dai.node.XLinkOut)

xout.setStreamName("disparity")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(lr_check)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(subpixel)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)
depth.disparity.link(xout.input)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the disparity frames from the outputs
    # defined above
    q = device.getOutputQueue(name="disparity", maxSize=4, blocking=False)

    while True:
        inDisparity = q.get()  # blocking call, will wait until a new data has
        # arrived
        frame = inDisparity.getFrame()

        # Normalization for better visualization
        ratio = 255 / depth.initialConfig.getMaxDisparity()
        frame = (frame * ratio).astype(np.uint8)
        frame_color = cv2.applyColorMap(frame, cv2.COLORMAP_JET)

        # Simulate object movement by updating the object's center position
        OBJ_CENTER_Y += move_direction  # Move the bounding box vertically
        if OBJ_CENTER_Y >= 200 or OBJ_CENTER_Y <= 50:  # Change dir at bounds
            move_direction *= -1

        # Correct the object center pixel using distortion coefficients
        OBJ_CENTER_CORRECTED = undistort_points(OBJ_CENTER_X, OBJ_CENTER_Y)

        # Calculate z_world
        z_world = calculate_z_world(DISP)

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
        obj_center_cam = np.array(
            [cam_center_x, cam_center_y, cam_center_z], dtype=np.float32
        )

        # Draw a bounding box around the moving object center
        cv2.rectangle(
            frame_color,
            (OBJ_CENTER_X - 15, OBJ_CENTER_Y - 15),
            (OBJ_CENTER_X + 15, OBJ_CENTER_Y + 15),
            (255, 0, 0),
            2,
        )

        # Display the frame with the bounding box
        cv2.imshow("Disparity Color", frame_color)

        # Print the object center in world space continuously
        print(f"Object center in camera frame: {obj_center_cam}")

        if cv2.waitKey(1) == ord("q"):
            break

# --- Print Final Results ---
print(f"Final Object center in world space: {obj_center_cam}")
