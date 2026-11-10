"""Quaternions: attitude as a unit quaternion, plus its exponential and logarithm maps.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * Arrays store a quaternion scalar-LAST: q = [x, y, z, w], the same order as
    ROS geometry_msgs/Quaternion and SciPy's Rotation.as_quat(). Below, the
    vector part is qv = [x, y, z] and the scalar part is qw = w.
  * Hamilton product, written p (x) q. Composition reads left to right as
    frames: q_world_from_c = q_world_from_b (x) q_b_from_c, and
    R(p (x) q) = R(p) @ R(q).
  * An attitude q is the body-to-world rotation: v_world = R(q) @ v_body.
  * q and -q are the same rotation. Functions that return a "canonical"
    quaternion pick the hemisphere w >= 0 and say so.
  * Small attitude errors are right (body-frame) perturbations:
    q_true = q (x) Exp(dtheta), dtheta a rotation vector in the body frame (rad).
  * In the full state x, shape (13,), q is x[3:7]; in the error state dx,
    shape (12,), dtheta is dx[3:6]. The functions here take q and dtheta
    directly, never x or dx.
  * No Euler angles anywhere in this package. Angles in radians.

Exp maps a rotation vector theta * k (angle theta about unit axis k) to a unit
quaternion; Log is its inverse. These two maps are how the error-state filter
moves between the 3-vector attitude error and the 4-number attitude.

Every function returns NumPy arrays of float. Do not use scipy in this file:
the tests use scipy as an independent reference.
"""
from __future__ import annotations

import numpy as np

SMALL_ANGLE = 1e-10
"""Below this, quat_exp and quat_log switch to their Taylor (small-angle) forms.

quat_exp compares |rotvec| (rad) with it; quat_log compares the norm of the
vector part |[x, y, z]| with it. Both avoid dividing by a number near zero.
"""


def quat_identity() -> np.ndarray:
    """The identity rotation as a quaternion.

    Returns
    -------
    np.ndarray, shape (4,)
        [0.0, 0.0, 0.0, 1.0] (scalar-last). A new array on every call, so the
        caller may modify it in place.
    """
    raise NotImplementedError("quat_identity")


def quat_normalize(q) -> np.ndarray:
    """Scale ``q`` to unit length: q / |q|.

    Parameters
    ----------
    q : array-like, shape (4,)
        Any non-zero quaternion [x, y, z, w].

    Returns
    -------
    np.ndarray, shape (4,)
        Unit-length quaternion. The sign is kept: no hemisphere flip, so a
        q with w < 0 stays with w < 0. The input is not modified.

    Raises
    ------
    ValueError
        If ``q`` has zero length (|q| == 0).
    """
    raise NotImplementedError("quat_normalize")


def quat_conjugate(q) -> np.ndarray:
    """Conjugate of ``q``: the vector part negated, [-x, -y, -z, w].

    For a unit quaternion the conjugate is the inverse rotation:
    q (x) conj(q) = [0, 0, 0, 1] and R(conj(q)) = R(q).T. If q is body-to-world,
    conj(q) is world-to-body.

    Parameters
    ----------
    q : array-like, shape (4,)
        Any quaternion [x, y, z, w]. It is not normalized here.

    Returns
    -------
    np.ndarray, shape (4,)
    """
    raise NotImplementedError("quat_conjugate")


def quat_multiply(p, q) -> np.ndarray:
    """Hamilton product p (x) q, both scalar-last.

    In vector form, with p = [pv, pw] and q = [qv, qw]:
        w  = pw * qw - pv . qv
        v  = pw * qv + qw * pv + pv x qv
    Component by component:
        x = pw*qx + px*qw + py*qz - pz*qy
        y = pw*qy - px*qz + py*qw + pz*qx
        z = pw*qz + px*qy - py*qx + pz*qw
        w = pw*qw - px*qx - py*qy - pz*qz

    The product is associative but NOT commutative. As rotations,
    R(p (x) q) = R(p) @ R(q): q acts on a vector first, then p.
    Check: [1, 0, 0, 0] (x) [0, 1, 0, 0] = [0, 0, 1, 0] (i j = k).

    Parameters
    ----------
    p, q : array-like, shape (4,)
        Quaternions [x, y, z, w]. Neither has to be unit.

    Returns
    -------
    np.ndarray, shape (4,)
        p (x) q. Not normalized: the product of unit quaternions is unit up to
        rounding, and callers normalize when drift matters.
    """
    raise NotImplementedError("quat_multiply")


def quat_to_R(q) -> np.ndarray:
    """Rotation matrix of a quaternion: v_world = R @ v_body when q is body-to-world.

    With q normalized first (so a slightly drifted q still gives an
    orthonormal R):
        R = (w^2 - qv . qv) I + 2 qv qv^T + 2 w [qv]x
    which, written out, is
        [[1 - 2(y^2 + z^2),   2(xy - zw),         2(xz + yw)      ],
         [2(xy + zw),         1 - 2(x^2 + z^2),   2(yz - xw)      ],
         [2(xz - yw),         2(yz + xw),         1 - 2(x^2 + y^2)]]

    q and -q give the same R. quat_to_R(quat_exp(angle * k)) is the rotation
    by ``angle`` about the unit axis k (Rodrigues' formula).

    Parameters
    ----------
    q : array-like, shape (4,)
        Non-zero quaternion [x, y, z, w]; normalized here.

    Returns
    -------
    np.ndarray, shape (3, 3)
        A rotation matrix (R.T @ R = I, det R = +1).

    Raises
    ------
    ValueError
        If ``q`` has zero length.
    """
    raise NotImplementedError("quat_to_R")


def R_to_quat(R) -> np.ndarray:
    """Unit quaternion of a rotation matrix (the inverse of quat_to_R), by Shepperd's method.

    Four expressions each give four times one squared component:
        4 w^2 = 1 + R[0,0] + R[1,1] + R[2,2]
        4 x^2 = 1 + R[0,0] - R[1,1] - R[2,2]
        4 y^2 = 1 - R[0,0] + R[1,1] - R[2,2]
        4 z^2 = 1 - R[0,0] - R[1,1] + R[2,2]
    Pick the LARGEST of the four (it is at least 1, because the four sum to 4).
    The chosen component is half its positive square root (so it is at least
    0.5). Read the other three components from the off-diagonal sums and
    differences, dividing by 4 times the chosen component:
        R[2,1] - R[1,2] = 4 x w      R[0,1] + R[1,0] = 4 x y
        R[0,2] - R[2,0] = 4 y w      R[0,2] + R[2,0] = 4 x z
        R[1,0] - R[0,1] = 4 z w      R[1,2] + R[2,1] = 4 y z
    For example, if w is chosen: w = 0.5 * sqrt(1 + t),
    x = (R[2,1] - R[1,2]) / (4 w), y = (R[0,2] - R[2,0]) / (4 w),
    z = (R[1,0] - R[0,1]) / (4 w). Choosing the largest keeps the divisor far
    from zero, so the result is accurate for every rotation, including
    angles near pi where w is near 0.

    Finally, negate the quaternion if w < 0 and normalize it to remove rounding.
    For an exact half turn w = 0, and q and -q are both acceptable.

    Parameters
    ----------
    R : array-like, shape (3, 3)
        A rotation matrix (v_world = R @ v_body). It may carry rounding error
        of about 1e-15; that must not produce NaN (the chosen square-root
        argument is at least 1, so it never goes negative). R is not checked:
        use rotations.is_rotation first if the input is untrusted.

    Returns
    -------
    np.ndarray, shape (4,)
        Unit quaternion [x, y, z, w] with w >= 0, so quat_to_R of it
        reproduces R. For a rotation by exactly pi, w = 0 and both q and -q
        satisfy w >= 0; either is correct.
    """
    raise NotImplementedError("R_to_quat")


def quat_exp(rotvec) -> np.ndarray:
    """Exp map: unit quaternion of the rotation vector ``rotvec`` = theta * k.

    The result rotates by theta = |rotvec| radians about the unit axis
    k = rotvec / theta (right-hand rule):
        q = [sin(theta / 2) * k, cos(theta / 2)]

    Small-angle form, used when theta < SMALL_ANGLE (the axis is undefined at
    theta = 0, and rotvec / theta would divide by a tiny number):
        q = [(0.5 - theta^2 / 48) * rotvec, 1 - theta^2 / 8]
    At these sizes this equals [0.5 * rotvec, 1.0] to double precision.
    rotvec = [0, 0, 0] gives exactly the identity [0, 0, 0, 1].

    No hemisphere flip: for pi < theta < 3*pi the result has w < 0, which is
    the same rotation as its negation. quat_log(quat_exp(r)) == r for |r| < pi.

    Check value: quat_exp([0, 0, pi/2]) = [0, 0, 0.70710678, 0.70710678], a
    quarter turn about +z, whose R maps body +x to world +y.

    Parameters
    ----------
    rotvec : array-like, shape (3,)
        Rotation vector in radians, any real 3-vector. In the filter it is a
        body-frame attitude error dtheta, or omega * dt for a body angular
        rate omega (rad/s).

    Returns
    -------
    np.ndarray, shape (4,)
        Unit quaternion [x, y, z, w].
    """
    raise NotImplementedError("quat_exp")


def quat_log(q) -> np.ndarray:
    """Log map: rotation vector theta * k of the rotation ``q`` (the inverse of quat_exp).

    Steps:
      1. Normalize q.
      2. If w < 0, negate q (same rotation), so w >= 0 and the angle is at
         most pi.
      3. With s = |qv| = sin(theta / 2) and w = cos(theta / 2):
             theta = 2 * atan2(s, w)       (in [0, pi])
             rotvec = theta * qv / s
         Use atan2, not acos(w): acos loses accuracy near w = 1.
      4. Instead of step 3, when s < SMALL_ANGLE (qv / s would divide by a
         tiny number), use the small-angle form rotvec = 2 * qv / w. Here
         w is close to 1, so the division is safe.

    q and -q give the same rotation vector, except for a rotation by exactly
    pi (w = 0), where step 2 cannot pick a side: rotvec and -rotvec then
    describe the same rotation and either may be returned.

    Parameters
    ----------
    q : array-like, shape (4,)
        Non-zero quaternion [x, y, z, w]; normalized here.

    Returns
    -------
    np.ndarray, shape (3,)
        Rotation vector, radians, with |rotvec| in [0, pi]. The identity gives
        [0, 0, 0].

    Raises
    ------
    ValueError
        If ``q`` has zero length.
    """
    raise NotImplementedError("quat_log")


def quat_rotate(q, v) -> np.ndarray:
    """Rotate the vector ``v`` by ``q``: R(q) @ v.

    Equal to the vector part of q (x) [v, 0] (x) conj(q) (scalar-last pure
    quaternion [vx, vy, vz, 0]), and to quat_to_R(q) @ v. Either form is
    fine.

    When q is the body-to-world attitude, this maps a body-frame vector to
    the world frame; quat_rotate(quat_conjugate(q), v) maps world to body.

    Parameters
    ----------
    q : array-like, shape (4,)
        Non-zero quaternion [x, y, z, w]; normalized here.
    v : array-like, shape (3,)
        Any 3-vector, in the frame q rotates from.

    Returns
    -------
    np.ndarray, shape (3,)
        The rotated vector; same length as ``v``.

    Raises
    ------
    ValueError
        If ``q`` has zero length.
    """
    raise NotImplementedError("quat_rotate")


def quat_error(q_ref, q) -> np.ndarray:
    """Body-frame rotation vector dtheta that takes ``q_ref`` to ``q``.

    Defined by the right (body-frame) perturbation
        q = q_ref (x) Exp(dtheta)
    so
        dtheta = Log(conj(q_ref) (x) q)
    computed with quat_conjugate, quat_multiply and quat_log. Equivalently,
    R(q) = R(q_ref) @ R(Exp(dtheta)): dtheta is expressed in the body axes of
    q_ref, not in the world frame.

    Properties:
      * quat_error(q, q) = [0, 0, 0].
      * Sign-blind: q or q_ref may be replaced by its negation without
        changing the result (quat_log works in the w >= 0 hemisphere),
        except when the two differ by exactly pi.
      * quat_error(q_ref, q) = -quat_error(q, q_ref), except at an angle of pi.
      * quat_error(q_ref, quat_multiply(q_ref, quat_exp(d))) == d for |d| < pi.

    Used by state.state_minus(x1, x0) as quat_error(q0, q1), and by the IMU
    attitude residual as quat_error(z_pred, z).

    Parameters
    ----------
    q_ref : array-like, shape (4,)
        Reference attitude [x, y, z, w], body to world. Non-zero.
    q : array-like, shape (4,)
        Attitude [x, y, z, w], body to world. Non-zero.

    Returns
    -------
    np.ndarray, shape (3,)
        dtheta in radians, body frame of q_ref, |dtheta| in [0, pi].

    Raises
    ------
    ValueError
        If either quaternion has zero length.
    """
    raise NotImplementedError("quat_error")
