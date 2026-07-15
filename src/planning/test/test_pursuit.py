import numpy as np
from planning.utils.pursuit import pursuit_target, step_toward, arrived


def test_pursuit_target_standoff_point():
    target, yaw = pursuit_target(
        np.array([10.0, 0.0, -1.0]), np.array([0.0, 0.0, -1.0]), standoff=2.0
    )
    assert np.allclose(target, [8.0, 0.0, -1.0])
    assert abs(yaw) < 1e-9


def test_pursuit_target_zero_standoff_is_goal():
    target, yaw = pursuit_target(
        np.array([3.0, 4.0, -1.0]), np.array([0.0, 0.0, -1.0]), standoff=0.0
    )
    assert np.allclose(target, [3.0, 4.0, -1.0])
    assert abs(yaw - np.degrees(np.arctan2(4.0, 3.0))) < 1e-9


def test_pursuit_target_inside_standoff_holds():
    # Already closer than standoff: target is current position (backing off
    # isn't worth thrashing), still faces the goal.
    target, yaw = pursuit_target(
        np.array([1.0, 0.0, -1.0]), np.array([0.0, 0.0, -1.0]), standoff=2.0
    )
    assert np.allclose(target, [0.0, 0.0, -1.0])
    assert abs(yaw) < 1e-9


def test_step_toward_clamps_speed():
    out = step_toward(
        np.array([0.0, 0.0, 0.0]), np.array([10.0, 0.0, 0.0]), v_max=0.25, dt=1.0
    )
    assert np.allclose(out, [0.25, 0.0, 0.0])


def test_step_toward_does_not_overshoot():
    out = step_toward(
        np.array([0.0, 0.0, 0.0]), np.array([0.1, 0.0, 0.0]), v_max=0.25, dt=1.0
    )
    assert np.allclose(out, [0.1, 0.0, 0.0])


def test_arrived():
    assert arrived(np.array([1.0, 0.0, 0.0]), np.array([1.05, 0.0, 0.0]), tol=0.1)
    assert not arrived(np.array([1.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]), tol=0.1)


def test_arrival_uses_ekf_pose_when_available():
    from planning.utils.pursuit import arrived, arrival_pos

    target = [2.0, 0.0, -1.0]
    cmd = [2.0, 0.0, -1.0]      # command already at target
    ekf = [1.0, 0.0, -1.0]      # sub physically 1 m short
    assert not arrived(arrival_pos(ekf, cmd), target, 0.3)
    assert arrived(arrival_pos([1.9, 0.0, -1.0], cmd), target, 0.3)
    # no EKF yet -> fall back to commanded pose
    assert arrived(arrival_pos(None, cmd), target, 0.3)
