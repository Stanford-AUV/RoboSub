"""Tests for hsr_ekf.quaternions.

Quaternions are scalar-last, q = [x, y, z, w], the same order as SciPy's
Rotation.as_quat(), which these tests use as an independent reference.

Run from src/hsr_ekf:
    python3 -m pytest test/test_quaternions.py -q
"""
import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from hsr_ekf.rotations import is_rotation
from hsr_ekf.quaternions import (
    R_to_quat,
    quat_conjugate,
    quat_error,
    quat_exp,
    quat_identity,
    quat_log,
    quat_multiply,
    quat_normalize,
    quat_rotate,
    quat_to_R,
)

PI = np.pi
SEED = 106
S2 = np.sqrt(2.0) / 2.0


def random_quat(rng):
    q = rng.normal(size=4)
    return q / np.linalg.norm(q)


def same_rotation(p, q, atol=1e-12):
    """q and -q are the same rotation."""
    p, q = np.asarray(p), np.asarray(q)
    return np.allclose(p, q, atol=atol) or np.allclose(p, -q, atol=atol)


# ---------------------------------------------------------------------------
# quat_identity, quat_normalize, quat_conjugate
# ---------------------------------------------------------------------------


def test_algebra_identity_is_scalar_last_one():
    q = quat_identity()
    assert np.asarray(q).shape == (4,)
    assert np.allclose(q, [0.0, 0.0, 0.0, 1.0])


def test_algebra_normalize_gives_unit_length_and_keeps_direction():
    q = np.array([1.0, -2.0, 3.0, 4.0])
    n = quat_normalize(q)
    assert np.isclose(np.linalg.norm(n), 1.0, atol=1e-15)
    assert np.allclose(n, q / np.linalg.norm(q), atol=1e-15)


def test_algebra_normalize_rejects_zero():
    with pytest.raises(ValueError):
        quat_normalize([0.0, 0.0, 0.0, 0.0])


def test_algebra_conjugate_flips_vector_part_only():
    assert np.allclose(quat_conjugate([0.1, -0.2, 0.3, 0.9]), [-0.1, 0.2, -0.3, 0.9])


def test_algebra_normalize_leaves_input_alone_and_returns_a_new_array():
    q = np.array([1.0, -2.0, 3.0, 4.0])
    before = q.copy()
    n = quat_normalize(q)
    assert np.array_equal(q, before), "normalize modified the caller's array"
    assert not np.shares_memory(n, q)
    u = np.array([0.0, 0.0, 0.0, 1.0])  # already exactly unit
    nu = quat_normalize(u)
    assert not np.shares_memory(nu, u), "normalize returned the caller's array"


def test_algebra_normalize_accepts_integer_arrays_and_lists():
    for q in ([1, 2, 2, 4], np.array([1, 2, 2, 4])):
        n = quat_normalize(q)
        assert isinstance(n, np.ndarray) and n.dtype == np.float64
        assert np.allclose(n, [0.2, 0.4, 0.4, 0.8], rtol=0.0, atol=1e-15)


def test_algebra_normalize_keeps_sign_and_double_precision():
    q = np.array([1.0, -2.0, 3.0, -4.0])
    n = quat_normalize(q)
    assert n.dtype == np.float64
    assert np.allclose(n, q / np.linalg.norm(q), rtol=0.0, atol=1e-15)
    assert np.allclose(quat_normalize([0.0, 0.0, 0.0, -2.0]), [0.0, 0.0, 0.0, -1.0])


def test_algebra_normalize_small_but_nonzero_quaternion():
    q = 1e-6 * np.array([1.0, -2.0, 3.0, 4.0])
    assert np.allclose(quat_normalize(q), q / np.linalg.norm(q), rtol=0.0, atol=1e-15)


def test_algebra_identity_is_a_fresh_float_array_each_call():
    a = quat_identity()
    assert isinstance(a, np.ndarray) and a.dtype == np.float64
    a[:3] = [0.1, 0.2, 0.3]  # an int array would silently truncate these to 0
    assert np.allclose(a[:3], [0.1, 0.2, 0.3])
    assert np.allclose(quat_identity(), [0.0, 0.0, 0.0, 1.0]), "identity is shared state"


def test_algebra_conjugate_and_multiply_return_float_ndarrays():
    q = np.array([1.0, -2.0, 3.0, 4.0])
    before = q.copy()
    c = quat_conjugate(q)
    assert np.array_equal(q, before)
    for out in (c, quat_conjugate([1, 2, 3, 4]), quat_multiply(q, q),
                quat_multiply([0, 0, 0, 1], [1, 0, 0, 0])):
        assert isinstance(out, np.ndarray) and out.shape == (4,) and out.dtype == np.float64


# ---------------------------------------------------------------------------
# quat_multiply
# ---------------------------------------------------------------------------


def test_algebra_multiply_unit_rules_i_j_k():
    i, j, k = [1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0]
    assert np.allclose(quat_multiply(i, j), k)
    assert np.allclose(quat_multiply(j, k), i)
    assert np.allclose(quat_multiply(k, i), j)
    assert np.allclose(quat_multiply(j, i), -np.asarray(k))
    assert np.allclose(quat_multiply(i, i), [0.0, 0.0, 0.0, -1.0])


def test_algebra_multiply_identity_changes_nothing():
    rng = np.random.default_rng(SEED)
    q = random_quat(rng)
    assert np.allclose(quat_multiply(q, quat_identity()), q)
    assert np.allclose(quat_multiply(quat_identity(), q), q)


def test_algebra_multiply_by_conjugate_gives_norm_squared():
    q = np.array([1.0, -2.0, 3.0, 4.0])
    assert np.allclose(quat_multiply(q, quat_conjugate(q)), [0.0, 0.0, 0.0, 30.0])


def test_algebra_multiply_matches_scipy_composition():
    rng = np.random.default_rng(SEED)
    for _ in range(100):
        p, q = random_quat(rng), random_quat(rng)
        expected = (Rotation.from_quat(p) * Rotation.from_quat(q)).as_quat()
        assert same_rotation(quat_multiply(p, q), expected)


def test_algebra_multiply_is_not_commutative():
    p = [0.0, 0.0, S2, S2]  # 90 deg about z
    q = [S2, 0.0, 0.0, S2]  # 90 deg about x
    assert not np.allclose(quat_multiply(p, q), quat_multiply(q, p))


def test_algebra_two_quarter_turns_make_a_third_turn_about_the_diagonal():
    # 90 deg left, then 90 deg about the sub's own nose.
    pq = quat_multiply([0.0, 0.0, S2, S2], [S2, 0.0, 0.0, S2])
    assert np.allclose(pq, [0.5, 0.5, 0.5, 0.5], atol=1e-12)


# ---------------------------------------------------------------------------
# quat_to_R and quat_rotate
# ---------------------------------------------------------------------------


def test_to_R_matches_scipy():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        q = random_quat(rng)
        assert np.allclose(quat_to_R(q), Rotation.from_quat(q).as_matrix(), atol=1e-12)


def test_to_R_of_identity_is_identity():
    assert np.allclose(quat_to_R([0.0, 0.0, 0.0, 1.0]), np.eye(3))


def test_to_R_same_for_q_and_minus_q():
    rng = np.random.default_rng(SEED)
    q = random_quat(rng)
    assert np.allclose(quat_to_R(q), quat_to_R(-q), atol=1e-14)


def test_to_R_quarter_turn_left_points_nose_along_world_left():
    R = quat_to_R([0.0, 0.0, S2, S2])
    assert np.allclose(R, [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], atol=1e-12)


def test_to_R_positive_turn_about_y_puts_nose_down():
    h = np.radians(10.0) / 2.0
    nose = quat_to_R([0.0, np.sin(h), 0.0, np.cos(h)]) @ [1.0, 0.0, 0.0]
    assert nose[2] < 0.0, "a positive turn about +y must put the nose DOWN in z-up frames"
    assert np.allclose(nose, [np.cos(2 * h), 0.0, -np.sin(2 * h)], atol=1e-12)


def test_to_R_combined_turn_gives_the_axis_cycling_matrix():
    R = quat_to_R([0.5, 0.5, 0.5, 0.5])
    assert np.allclose(R, [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], atol=1e-12)


def test_to_R_of_product_is_product_of_matrices():
    rng = np.random.default_rng(SEED)
    for _ in range(50):
        p, q = random_quat(rng), random_quat(rng)
        assert np.allclose(quat_to_R(quat_multiply(p, q)), quat_to_R(p) @ quat_to_R(q), atol=1e-12)


def test_rotate_matches_matrix_and_keeps_length():
    rng = np.random.default_rng(SEED)
    for _ in range(50):
        q, v = random_quat(rng), rng.normal(size=3)
        out = quat_rotate(q, v)
        assert np.asarray(out).shape == (3,)
        assert np.allclose(out, quat_to_R(q) @ v, atol=1e-12)
        assert np.isclose(np.linalg.norm(out), np.linalg.norm(v), atol=1e-12)


def test_rotate_dvl_example_turned_left():
    # Turned 90 deg left, moving forward 0.5 m/s: that is +y in the pool.
    assert np.allclose(quat_rotate([0.0, 0.0, S2, S2], [0.5, 0.0, 0.0]), [0.0, 0.5, 0.0], atol=1e-12)


def test_to_R_normalizes_a_drifted_quaternion():
    rng = np.random.default_rng(SEED)
    q = random_quat(rng)
    assert np.allclose(quat_to_R(2.0 * q), quat_to_R(q), atol=1e-14)
    assert is_rotation(quat_to_R(1.01 * q))


def test_rotate_normalizes_a_drifted_quaternion():
    rng = np.random.default_rng(SEED)
    q, v = random_quat(rng), rng.normal(size=3)
    assert np.isclose(np.linalg.norm(quat_rotate(3.0 * q, v)), np.linalg.norm(v), atol=1e-12)


def test_to_R_and_rotate_reject_zero_quaternion():
    with pytest.raises(ValueError):
        quat_to_R([0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        quat_rotate([0.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# R_to_quat
# ---------------------------------------------------------------------------


def test_R_to_quat_round_trip_and_w_nonnegative():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        q = random_quat(rng)
        q2 = R_to_quat(Rotation.from_quat(q).as_matrix())
        assert np.asarray(q2).shape == (4,)
        assert np.isclose(np.linalg.norm(q2), 1.0, atol=1e-12)
        assert q2[3] >= 0.0
        assert same_rotation(q2, q, atol=1e-10)


def test_R_to_quat_identity():
    assert np.allclose(R_to_quat(np.eye(3)), [0.0, 0.0, 0.0, 1.0])


@pytest.mark.parametrize("k", [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 2.0, 3.0]])
def test_R_to_quat_half_turns(k):
    # w = 0 for a half turn: dividing by 4w would blow up.
    k = np.asarray(k) / np.linalg.norm(k)
    R = 2.0 * np.outer(k, k) - np.eye(3)
    q = R_to_quat(R)
    assert not np.any(np.isnan(q)), "returned NaN"
    assert np.isclose(q[3], 0.0, atol=1e-12)
    assert same_rotation(q, np.append(k, 0.0), atol=1e-12)
    assert np.allclose(quat_to_R(q), R, atol=1e-12)


def test_R_to_quat_axis_cycling_matrix():
    R = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
    assert np.allclose(R_to_quat(R), [0.5, 0.5, 0.5, 0.5], atol=1e-12)


@pytest.mark.parametrize("gap", [1e-2, 1e-3, 1e-5, 1e-7, 1e-9])
def test_R_to_quat_keeps_full_precision_near_a_half_turn(gap):
    # Just short of 180 deg, w is close to 0, so 1 + trace(R) = 4 w^2 is a
    # cancellation of nearly equal numbers: the answer must come from the largest
    # component instead.
    rng = np.random.default_rng(SEED)
    for _ in range(50):
        k = rng.normal(size=3)
        k /= np.linalg.norm(k)
        rot = Rotation.from_rotvec((PI - gap) * k)
        q2 = R_to_quat(rot.as_matrix())
        assert np.isclose(np.linalg.norm(q2), 1.0, atol=1e-12)
        assert same_rotation(q2, rot.as_quat(), atol=1e-12)
        assert np.allclose(quat_to_R(q2), rot.as_matrix(), atol=1e-12)


# ---------------------------------------------------------------------------
# quat_exp and quat_log
# ---------------------------------------------------------------------------


def test_exp_quarter_turn_about_z():
    assert np.allclose(quat_exp([0.0, 0.0, PI / 2]), [0.0, 0.0, S2, S2], atol=1e-12)


def test_exp_uses_the_half_angle():
    q = quat_exp([0.3, 0.0, 0.0])
    assert np.allclose(q, [np.sin(0.15), 0.0, 0.0, np.cos(0.15)], atol=1e-14)


def test_exp_matches_scipy():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        phi = rng.normal(size=3)
        assert same_rotation(quat_exp(phi), Rotation.from_rotvec(phi).as_quat(), atol=1e-12)


def test_exp_of_zero_and_tiny_vectors():
    assert np.allclose(quat_exp([0.0, 0.0, 0.0]), [0.0, 0.0, 0.0, 1.0])
    q = quat_exp([1e-13, -2e-13, 3e-13])
    assert not np.any(np.isnan(q)), "returned NaN for a tiny rotation"
    assert np.allclose(q, [5e-14, -1e-13, 1.5e-13, 1.0], atol=1e-20)


def test_log_inverts_exp():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        k = rng.normal(size=3)
        phi = rng.uniform(0.0, PI - 1e-3) * k / np.linalg.norm(k)
        assert np.allclose(quat_log(quat_exp(phi)), phi, atol=1e-10)


def test_log_is_same_for_q_and_minus_q():
    rng = np.random.default_rng(SEED)
    for _ in range(50):
        q = random_quat(rng)
        assert np.allclose(quat_log(q), quat_log(-q), atol=1e-12)


def test_log_angle_never_exceeds_pi():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        assert np.linalg.norm(quat_log(random_quat(rng))) <= PI + 1e-12


def test_log_of_identity_and_minus_identity_is_zero():
    assert np.allclose(quat_log([0.0, 0.0, 0.0, 1.0]), 0.0)
    assert np.allclose(quat_log([0.0, 0.0, 0.0, -1.0]), 0.0)


def test_log_tiny_and_no_nan_when_w_rounds_past_one():
    phi = quat_log([5e-14, 0.0, 0.0, 1.0])
    assert np.allclose(phi, [1e-13, 0.0, 0.0], atol=1e-20)
    phi = quat_log([0.0, 0.0, 0.0, 1.0000000000000002])
    assert not np.any(np.isnan(phi)) and np.allclose(phi, 0.0)


def test_log_small_rotation_keeps_full_precision():
    # Small attitude errors are the filter's everyday case. 2*arccos(w) loses them
    # entirely, because w rounds to exactly 1.0; atan2 keeps every digit.
    for theta in [3e-9, 1e-7, 1e-5]:
        phi = np.array([theta, -2.0 * theta, 0.5 * theta])
        assert np.allclose(quat_log(quat_exp(phi)), phi, rtol=1e-6, atol=0.0)


def test_log_half_turn():
    phi = quat_log([0.0, 1.0, 0.0, 0.0])
    assert np.allclose(np.abs(phi), [0.0, PI, 0.0], atol=1e-12)


def test_log_matches_scipy_rotvec():
    rng = np.random.default_rng(SEED)
    for _ in range(200):
        q = random_quat(rng)
        expected = Rotation.from_quat(q).as_rotvec()
        assert np.allclose(quat_log(q), expected, atol=1e-10)


def test_exp_small_and_moderate_angles_are_exact_and_unit_length():
    # rtol=0: the default rtol=1e-5 of np.allclose would hide a too-large
    # small-angle cutoff.
    rng = np.random.default_rng(7)
    for theta in [1e-9, 1e-7, 1e-5, 1e-4, 1e-3, 5e-3, 1e-2, 3e-2, 0.1, 0.3]:
        k = rng.normal(size=3)
        k /= np.linalg.norm(k)
        q = np.asarray(quat_exp(theta * k))
        expected = np.append(np.sin(theta / 2) * k, np.cos(theta / 2))
        assert np.allclose(q, expected, rtol=0.0, atol=1e-15), theta
        assert abs(np.linalg.norm(q) - 1.0) < 1e-15, theta


def test_log_small_angles_keep_full_relative_precision():
    rng = np.random.default_rng(8)
    for theta in [1e-9, 2e-5, 1e-4, 3e-4, 1e-3, 3e-3, 1e-2, 3e-2, 0.1]:
        k = rng.normal(size=3)
        k /= np.linalg.norm(k)
        q = np.append(np.sin(theta / 2) * k, np.cos(theta / 2))
        assert np.allclose(quat_log(q), theta * k, rtol=1e-13, atol=0.0), theta


def test_log_near_half_turn_is_precise_and_never_nan():
    # Facing almost exactly backwards is ordinary in a pool.
    a = np.array([1.0, 2.0, 3.0]) / np.sqrt(14.0)
    for eps in [1e-3, 1e-5, 1e-7]:
        q = np.append(a * np.cos(eps / 2), np.sin(eps / 2))
        assert np.allclose(quat_log(q), (PI - eps) * a, rtol=0.0, atol=1e-13), eps
    # After normalizing, |v| of an exact half turn can round to 1 + 2.2e-16.
    rng = np.random.default_rng(0)
    for _ in range(2000):
        q = quat_normalize(np.append(rng.normal(size=3), 0.0))
        phi = np.asarray(quat_log(q))
        assert not np.any(np.isnan(phi)), q
        assert np.isclose(np.linalg.norm(phi), PI, rtol=0.0, atol=1e-12)
        assert np.allclose(Rotation.from_rotvec(phi).as_matrix(),
                           Rotation.from_quat(q).as_matrix(), rtol=0.0, atol=1e-12)


def test_log_normalizes_and_rejects_zero():
    phi = np.array([0.1, -0.2, 0.3])
    assert np.allclose(quat_log(2.0 * quat_exp(phi)), phi, atol=1e-14)
    with pytest.raises(ValueError):
        quat_log([0.0, 0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# quat_error
# ---------------------------------------------------------------------------


def test_error_of_a_quaternion_with_itself_is_zero():
    rng = np.random.default_rng(SEED)
    q = random_quat(rng)
    assert np.allclose(quat_error(q, q), 0.0, atol=1e-12)


def test_error_recovers_the_body_frame_perturbation():
    rng = np.random.default_rng(SEED)
    for _ in range(50):
        q_ref = random_quat(rng)
        dtheta = 0.05 * rng.normal(size=3)
        q = quat_multiply(q_ref, quat_exp(dtheta))  # small turn about the BODY axes
        assert np.allclose(quat_error(q_ref, q), dtheta, atol=1e-12)


def test_error_is_not_the_world_frame_perturbation():
    q_ref = quat_exp([0.0, 0.0, PI / 2])  # turned 90 deg left
    dtheta = np.array([0.01, 0.0, 0.0])
    q_world = quat_multiply(quat_exp(dtheta), q_ref)  # turn about the WORLD x axis instead
    assert not np.allclose(quat_error(q_ref, q_world), dtheta, atol=1e-6)


def test_error_ignores_the_sign_of_q():
    rng = np.random.default_rng(SEED)
    q_ref = random_quat(rng)
    q = quat_multiply(q_ref, quat_exp([0.02, -0.01, 0.03]))
    assert np.allclose(quat_error(q_ref, q), quat_error(q_ref, -q), atol=1e-12)
    assert np.allclose(quat_error(-q_ref, q), quat_error(q_ref, q), atol=1e-12)


def test_error_small_imu_example():
    q = [0.0, 0.0, np.sin(0.01), np.cos(0.01)]
    assert np.allclose(quat_error([0.0, 0.0, 0.0, 1.0], q), [0.0, 0.0, 0.02], atol=1e-14)


def test_error_rejects_zero_quaternion():
    with pytest.raises(ValueError):
        quat_error([0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    with pytest.raises(ValueError):
        quat_error([0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0])
