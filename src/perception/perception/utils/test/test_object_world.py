import numpy as np
from perception.utils.object_world import (
    camera_to_base,
    base_to_odom,
    DetectionFilter,
)


def test_camera_to_base_identity_mount():
    # Forward-looking camera, no offset: optical z (forward) -> base x.
    p = camera_to_base([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert np.allclose(p, [2.0, 0.0, 0.0])
    # optical x (right) -> base -y; optical y (down) -> base -z
    p = camera_to_base([0.5, 0.3, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    assert np.allclose(p, [2.0, -0.5, -0.3])


def test_camera_to_base_translation():
    p = camera_to_base([0.0, 0.0, 2.0], [0.1, -0.2, 0.05], [0.0, 0.0, 0.0])
    assert np.allclose(p, [2.1, -0.2, 0.05])


def test_camera_to_base_yawed_mount():
    # Camera yawed +90 deg (looking along base +y).
    p = camera_to_base([0.0, 0.0, 2.0], [0.0, 0.0, 0.0], [0.0, 0.0, 90.0])
    assert np.allclose(p, [0.0, 2.0, 0.0], atol=1e-9)


def test_base_to_odom():
    # Sub at (1, 2, -0.5), yawed 90 deg: base +x maps to odom +y.
    quat = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]  # yaw 90
    p = base_to_odom([2.0, 0.0, 0.0], [1.0, 2.0, -0.5], quat)
    assert np.allclose(p, [1.0, 4.0, -0.5], atol=1e-9)


def test_filter_median_then_ema_converges():
    f = DetectionFilter(window=3, alpha=0.5, max_jump_m=3.0, stale_sec=2.0)
    for i in range(10):
        f.update(np.array([4.0, 0.0, 0.0]), t=float(i) * 0.1)
    out = f.get(t=1.0)
    assert out is not None
    assert np.allclose(out, [4.0, 0.0, 0.0], atol=1e-6)


def test_filter_rejects_jump():
    f = DetectionFilter(window=3, alpha=0.5, max_jump_m=3.0, stale_sec=2.0)
    for i in range(5):
        f.update(np.array([4.0, 0.0, 0.0]), t=float(i) * 0.1)
    assert f.update(np.array([40.0, 0.0, 0.0]), t=0.6) is None
    assert np.allclose(f.get(t=0.7), [4.0, 0.0, 0.0], atol=1e-6)


def test_filter_stale_returns_none():
    f = DetectionFilter(window=3, alpha=0.5, max_jump_m=3.0, stale_sec=2.0)
    f.update(np.array([4.0, 0.0, 0.0]), t=0.0)
    assert f.get(t=1.0) is not None
    assert f.get(t=5.0) is None


def test_filter_empty_returns_none():
    f = DetectionFilter()
    assert f.get(t=0.0) is None
