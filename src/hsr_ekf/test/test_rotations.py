"""Tests for hsr_ekf.rotations.

Run from src/hsr_ekf:
    python3 -m pytest test/test_rotations.py -q
"""
import numpy as np
from scipy.spatial.transform import Rotation

from hsr_ekf.rotations import is_rotation, skew

SEED = 106


# ---------------------------------------------------------------------------
# skew
# ---------------------------------------------------------------------------


def test_skew_matches_cross_product():
    rng = np.random.default_rng(SEED)
    for _ in range(20):
        a = rng.normal(size=3)
        b = rng.normal(size=3)
        assert np.allclose(skew(a) @ b, np.cross(a, b), atol=1e-12)


def test_skew_is_skew_symmetric():
    S = skew(np.array([1.0, -2.0, 3.0]))
    assert S.shape == (3, 3)
    assert np.allclose(S.T, -S)
    assert np.allclose(np.diag(S), 0.0)


def test_skew_accepts_a_list():
    assert np.allclose(skew([0.0, 0.0, 1.0]) @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0])


def test_skew_of_a_vector_times_itself_is_zero():
    v = np.array([0.3, -1.2, 2.0])
    assert np.allclose(skew(v) @ v, 0.0, atol=1e-12)


# ---------------------------------------------------------------------------
# is_rotation
# ---------------------------------------------------------------------------


def test_is_rotation_accepts_identity_and_random_rotations():
    assert is_rotation(np.eye(3)) is True
    rng = np.random.default_rng(SEED)
    for _ in range(20):
        q = rng.normal(size=4)
        assert is_rotation(Rotation.from_quat(q / np.linalg.norm(q)).as_matrix()) is True


def test_is_rotation_rejects_a_mirror():
    assert is_rotation(np.diag([1.0, 1.0, -1.0])) is False


def test_is_rotation_rejects_stretch_and_shear():
    assert is_rotation(2.0 * np.eye(3)) is False
    shear = np.eye(3)
    shear[0, 1] = 0.1
    assert is_rotation(shear) is False


def test_is_rotation_rejects_wrong_shape():
    assert is_rotation(np.eye(2)) is False


def test_skew_of_integer_input_is_float():
    S = skew(np.array([1, 2, 3]))
    assert S.dtype == np.float64
    S *= 0.5
    assert np.allclose(S @ [1.0, 0.0, 0.0], 0.5 * np.cross([1, 2, 3], [1, 0, 0]))


def test_is_rotation_honours_tol():
    R = Rotation.from_rotvec([0.3, -0.2, 0.1]).as_matrix()
    R_bad = R.copy()
    R_bad[0, 0] += 1e-7  # orthogonality error of about 1e-7
    assert is_rotation(R_bad) is False, "the default tol must be tight"
    assert is_rotation(R_bad, tol=1e-5) is True, "the tol argument is ignored"
    R_ok = R.copy()
    R_ok[0, 0] += 1e-12
    assert is_rotation(R_ok) is True


def test_is_rotation_rejects_unit_columns_that_are_not_orthogonal():
    e = 1e-5
    R = np.column_stack([[1.0, 0.0, 0.0], [np.sin(e), np.cos(e), 0.0], [0.0, 0.0, 1.0]])
    assert is_rotation(R) is False


def test_is_rotation_rejects_nan_and_inf():
    assert is_rotation(np.full((3, 3), np.nan)) is False
    R = np.eye(3)
    R[1, 2] = np.nan
    assert is_rotation(R) is False
    R = np.eye(3)
    R[0, 0] = np.inf
    assert is_rotation(R) is False


def test_is_rotation_rejects_other_shapes_without_raising():
    assert is_rotation(np.eye(4)) is False
    assert is_rotation(np.eye(3)[:, :2]) is False
    assert is_rotation(np.eye(3).ravel()) is False
    assert is_rotation(np.eye(3)[None]) is False
