"""Rotations: the skew matrix and a rotation-matrix check.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up.
  * R maps body to world: v_world = R @ v_body, and v_body = R.T @ v_world.

No Euler angles in this package: attitude is carried as a quaternion
(hsr_ekf.quaternions), and R = quaternions.quat_to_R(q) when a matrix is
needed. This module holds the matrix helpers the filter uses: the skew
matrix, which appears in every rotation Jacobian, and a rotation check.

skew returns a float NumPy array; is_rotation returns a plain Python bool.
Do not use scipy in this file: the tests use scipy as an independent
reference.
"""
from __future__ import annotations

import numpy as np

def skew(v):
    """Return the skew-symmetric matrix [v]x, so that skew(v) @ b == np.cross(v, b).

    Parameters
    ----------
    v : array-like, shape (3,)
        Any 3-vector (a list or a NumPy array).

    Returns
    -------
    np.ndarray, shape (3, 3)
    """
    raise NotImplementedError("skew")


def is_rotation(R, tol: float = 1e-9) -> bool:
    """True if ``R`` is a 3x3 rotation matrix: R.T @ R == I and det(R) == +1, within ``tol``.

    Returns False (never raises) for a wrong shape, a mirror (det -1), a
    matrix that stretches or shears, or any NaN/inf entry. Check the shape is
    (3, 3) before any matrix arithmetic. The tolerance is absolute: every entry
    of R.T @ R - I and |det R - 1| must be within ``tol``.

    Returns a plain Python bool, not numpy.bool_ (wrap the result in bool(...)).
    """
    raise NotImplementedError("is_rotation")
