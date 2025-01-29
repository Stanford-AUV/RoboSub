# A list of 3D detections' points, for a multi-object 3D detector.

std_msgs/Header header

# A list of the detected proposals. A multi-proposal detector might generate
#   this list with many candidate detections generated from a single input.
Detection3DPoints[] detections