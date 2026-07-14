"""Small orientation helpers shared by planning nodes."""

from scipy.spatial.transform import Rotation


def override_yaw(quat_xyzw, yaw_deg):
    """Return the quaternion (xyzw) with its yaw replaced, roll/pitch kept."""
    roll, pitch, _ = Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=True)
    return Rotation.from_euler(
        "xyz", [roll, pitch, yaw_deg], degrees=True
    ).as_quat()
