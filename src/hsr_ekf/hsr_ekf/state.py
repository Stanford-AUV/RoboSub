"""State layout: index slices, construction, and the (+) / (-) operators.

The filter keeps two vectors. The full (nominal) state x has 13 entries and
holds the attitude as a 4-entry unit quaternion. The error state dx has 12
entries: the attitude error is a 3-entry rotation vector, because a rotation
has three degrees of freedom and the covariance must be 12 x 12 to stay
invertible. The two are connected by:

  x_true = x (+) dx         (state_plus: apply a small error to a state)
  dx     = x_true (-) x     (state_minus: measure the error between states)

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103). Not NED.
  * Attitude is a unit quaternion q = [x, y, z, w], scalar-last, Hamilton
    product, body to world: v_world = R(q) @ v_body. No Euler angles.
  * Full state x, shape (13,):
      x[0:3]   = p  position of the body origin, world frame, m
      x[3:7]   = q  attitude, body to world, scalar-last
      x[7:10]  = v  linear velocity, body frame, m/s
      x[10:13] = w  angular velocity, body frame, rad/s
  * Error state dx, shape (12,):
      dx[0:3]  = dp      world frame, m
      dx[3:6]  = dtheta  rotation vector, body frame, rad
      dx[6:9]  = dv      body frame, m/s
      dx[9:12] = dw      body frame, rad/s
  * Right (body-frame) perturbation of the attitude:
      q_true = q (x) Exp(dtheta)
    p, v and w are perturbed by plain addition, component by component.
  * SI units.

Governing equations:
  x (+) dx  = [p + dp,  normalize(q (x) Exp(dtheta)),  v + dv,  w + dw]
  x1 (-) x0 = [p1 - p0, Log(conj(q0) (x) q1),          v1 - v0, w1 - w0]

Implementations may use quat_normalize, quat_multiply, quat_exp and
quat_error from hsr_ekf.quaternions. Every function returns NumPy arrays
of float and does not modify its inputs.
"""
from __future__ import annotations

import numpy as np

STATE_DIM = 13
"""Length of the full state x = [p, q, v, w]."""

ERROR_DIM = 12
"""Length of the error state dx = [dp, dtheta, dv, dw]."""

P = slice(0, 3)
"""Full-state slice of the position p (world frame, m)."""

Q = slice(3, 7)
"""Full-state slice of the attitude quaternion q (body to world, [x, y, z, w])."""

V = slice(7, 10)
"""Full-state slice of the linear velocity v (body frame, m/s)."""

W = slice(10, 13)
"""Full-state slice of the angular velocity w (body frame, rad/s)."""

DP = slice(0, 3)
"""Error-state slice of the position error dp (world frame, m)."""

DTH = slice(3, 6)
"""Error-state slice of the attitude error dtheta (rotation vector, body frame, rad)."""

DV = slice(6, 9)
"""Error-state slice of the linear velocity error dv (body frame, m/s)."""

DW = slice(9, 12)
"""Error-state slice of the angular velocity error dw (body frame, rad/s)."""


def make_state(p, q, v, w) -> np.ndarray:
    """Pack the four parts into a full state vector x = [p, q, v, w].

    Parameters
    ----------
    p : array-like, shape (3,)
        Position of the body origin, world frame, m.
    q : array-like, shape (4,)
        Attitude, body to world, scalar-last [x, y, z, w]. Any non-zero
        length; it is normalized before it is stored, so [0, 0, 0, 2] is
        stored as the identity [0, 0, 0, 1].
    v : array-like, shape (3,)
        Linear velocity, body frame, m/s.
    w : array-like, shape (3,)
        Angular velocity, body frame, rad/s.

    Returns
    -------
    np.ndarray, shape (13,)
        x with x[P] = p, x[Q] = q / |q|, x[V] = v, x[W] = w. A new array of
        float; the inputs are not modified.

    Raises
    ------
    ValueError
        If any part has the wrong length, or if q has zero norm.
    """
    raise NotImplementedError("make_state")


def state_plus(x, dx) -> np.ndarray:
    """Apply an error-state vector to a full state: x (+) dx.

    x (+) dx = [p + dp,  normalize(q (x) Exp(dtheta)),  v + dv,  w + dw]

    Exp(dtheta) is the unit quaternion of a rotation by |dtheta| radians
    about dtheta (quat_exp). It multiplies on the RIGHT of q because dtheta
    is expressed in the body frame. Exp is exact for any angle, so a large
    dtheta is allowed; the result is renormalized to remove rounding drift.

    Parameters
    ----------
    x : array-like, shape (13,)
        Full state [p, q, v, w]; q must have unit norm (within about 1e-9).
    dx : array-like, shape (12,)
        Error state [dp, dtheta, dv, dw]: dp world frame (m), dtheta body-frame
        rotation vector (rad), dv body frame (m/s), dw body frame (rad/s).

    Returns
    -------
    np.ndarray, shape (13,)
        The perturbed state. A new array; x and dx are not modified.
        dx = 0 returns x (with q renormalized).

    Raises
    ------
    ValueError
        If x does not have shape (13,) or dx does not have shape (12,).
    """
    raise NotImplementedError("state_plus")


def state_minus(x1, x0) -> np.ndarray:
    """Error-state difference between two full states: x1 (-) x0.

    x1 (-) x0 = [p1 - p0,  quat_error(q0, q1),  v1 - v0,  w1 - w0]

    quat_error(q0, q1) = Log(conj(q0) (x) q1) is the body-frame rotation
    vector dtheta (in the body axes of q0) with q1 = q0 (x) Exp(dtheta). Its
    angle lies in [0, pi]. q1 or q0 may be replaced by its negation without
    changing the result, since q and -q describe the same rotation, except
    when the two attitudes differ by exactly pi (then either sign of dtheta
    may be returned).

    This is the inverse of state_plus:
      state_minus(state_plus(x, dx), x) == dx   (up to rounding, |dtheta| < pi)
      state_plus(x0, state_minus(x1, x0)) == x1 (up to the sign of q)
    At |dtheta| == pi the rotation-vector sign is ambiguous and only the
    second identity holds.

    Parameters
    ----------
    x1 : array-like, shape (13,)
        Full state [p, q, v, w]; q must have unit norm.
    x0 : array-like, shape (13,)
        Reference full state [p, q, v, w]; q must have unit norm.

    Returns
    -------
    np.ndarray, shape (12,)
        [dp, dtheta, dv, dw] in the frames listed in the module docstring.
        state_minus(x, x) is all zeros.

    Raises
    ------
    ValueError
        If x1 or x0 does not have shape (13,).
    """
    raise NotImplementedError("state_minus")
