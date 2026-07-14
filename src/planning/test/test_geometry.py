import numpy as np
from scipy.spatial.transform import Rotation

from planning.utils.geometry import override_yaw


def test_override_yaw_replaces_yaw_keeps_roll_pitch():
    quat = Rotation.from_euler("xyz", [5.0, -3.0, 120.0], degrees=True).as_quat()
    out = override_yaw(quat, 42.0)
    roll, pitch, yaw = Rotation.from_quat(out).as_euler("xyz", degrees=True)
    assert np.allclose([roll, pitch, yaw], [5.0, -3.0, 42.0], atol=1e-9)


def test_override_yaw_identity_when_yaw_matches():
    out = override_yaw([0.0, 0.0, 0.0, 1.0], 0.0)
    assert np.allclose(out, [0.0, 0.0, 0.0, 1.0], atol=1e-9)
