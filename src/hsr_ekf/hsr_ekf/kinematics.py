"""Kinematics: how position and attitude change with the body-frame velocities.

These are the geometric relations of the state x = [p, q, v, w], shape (13,):
x[0:3] = p, x[3:7] = q, x[7:10] = v, x[10:13] = w. The functions here take the
pieces, not x itself. They carry no mass, force, or drag; hsr_ekf.dynamics
supplies nu_dot = [v_dot; w_dot].

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103). Not NED.
  * Attitude is a unit quaternion q = [x, y, z, w], scalar-last, Hamilton
    product, body to world: v_world = R(q) @ v_body. No Euler angles.
  * p (position) is in the world frame, m. v (linear velocity, m/s) and
    w (angular velocity, rad/s) are in the body frame.
  * In this module the letter w alone is the angular velocity; the
    quaternion's scalar part is written q_w.
  * SI units.

Governing equations:
  p_dot   = R(q) v                    (world frame, m/s)
  q_dot   = 0.5 * q (x) [w, 0]        ([w, 0] = [wx, wy, wz, 0], scalar-last)
  v_point = v + w x r                 (body frame, point at body offset r)

Implementations may use quat_rotate and quat_multiply from
hsr_ekf.quaternions and skew from hsr_ekf.rotations. Every function
returns NumPy arrays of float and does not modify its inputs.
"""
from __future__ import annotations

import numpy as np


def position_rate(q, v) -> np.ndarray:
    """World-frame rate of change of position: p_dot = R(q) v.

    The velocity v is measured in the body frame; rotating it by the attitude
    gives the velocity of the body origin in the world frame.

    Parameters
    ----------
    q : array-like, shape (4,)
        Attitude, body to world, scalar-last [x, y, z, w]. Must have unit
        norm; it is neither checked nor renormalized here, so callers keep q
        normalized (a non-unit q gives a scaled or distorted result).
    v : array-like, shape (3,)
        Linear velocity of the body origin, body frame, m/s.

    Returns
    -------
    np.ndarray, shape (3,)
        p_dot = R(q) @ v, world frame, m/s. Equal to quat_rotate(q, v).
        For q = [0, 0, 0, 1] (identity) this is v itself; v = 0 gives zeros.
    """
    raise NotImplementedError("position_rate")


def quat_rate(q, w) -> np.ndarray:
    """Time derivative of the attitude quaternion: q_dot = 0.5 * q (x) [w, 0].

    The body-frame rate multiplies on the RIGHT of q because w is expressed in
    the body frame (a world-frame rate would multiply on the left:
    0.5 * [w_world, 0] (x) q).

    Written out with q = [q_x, q_y, q_z, q_w], q_v = [q_x, q_y, q_z] and
    w = [wx, wy, wz]:
        q_dot[0:3] = 0.5 * (q_w * w + q_v x w)
        q_dot[3]   = -0.5 * (q_v . w)

    Parameters
    ----------
    q : array-like, shape (4,)
        Attitude, body to world, scalar-last [x, y, z, w]. Must have unit
        norm; it is not renormalized here.
    w : array-like, shape (3,)
        Angular velocity, body frame, rad/s.

    Returns
    -------
    np.ndarray, shape (4,)
        q_dot, scalar-last, units 1/s. It is orthogonal to q
        (q . q_dot = 0 for any q), so the exact flow keeps |q| = 1; a discrete step
        (e.g. Euler) does not, so integrators renormalize after each step
        (hsr_ekf.integrate.quat_step uses the exact exponential step
        instead). w = 0 gives zeros.
    """
    raise NotImplementedError("quat_rate")


def point_velocity(v, w, r) -> np.ndarray:
    """Body-frame velocity of a point rigidly attached to the vehicle: v + w x r.

    Used for sensors mounted away from the body origin (lever arm), such as
    the DVL: the velocity it reports is that of its own mounting point, not
    of ``base_link``. The inverse relation, v = v_point - w x r, recovers the
    origin velocity from a sensor reading.

    Parameters
    ----------
    v : array-like, shape (3,)
        Linear velocity of the body origin, body frame, m/s.
    w : array-like, shape (3,)
        Angular velocity, body frame, rad/s.
    r : array-like, shape (3,)
        Position of the point relative to the body origin, body frame, m.

    Returns
    -------
    np.ndarray, shape (3,)
        v + np.cross(w, r) (equivalently v + skew(w) @ r), body frame, m/s.
        The order is w x r; r x w has the opposite sign.

        Example: v = 0, w = [0, 0, 1] rad/s (turning left about +z), and a
        point 0.1 m behind the origin, r = [-0.1, 0, 0], gives
        [0, -0.1, 0]: the tail swings to the right (-y). r = 0 or w = 0
        returns v unchanged.
    """
    raise NotImplementedError("point_velocity")
