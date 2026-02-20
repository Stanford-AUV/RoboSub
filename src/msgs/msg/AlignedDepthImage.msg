# Aligned RGB + depth and RGB camera intrinsics (Option A).
# Depth is aligned to the RGB frame (same resolution, pixel correspondence).
# Used by camera-agnostic object localizer for YOLO + depth lookup + deprojection.

sensor_msgs/Image rgb
sensor_msgs/Image depth
sensor_msgs/CameraInfo camera_info
builtin_interfaces/Time hardware_stamp
