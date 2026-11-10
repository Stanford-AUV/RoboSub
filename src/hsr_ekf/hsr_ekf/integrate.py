"""Integrate: one-step ODE integrators and an exact attitude step.

euler_step and rk4_step advance a plain vector ODE x_dot = f(x, u) by one
time step dt, with the input u held constant over the step (zero-order
hold). They treat every entry of x as an ordinary real number. Applied to
the full state [p, q, v, w], they move q off the unit sphere, so a process
model either renormalizes q after the step or advances the attitude
separately with quat_step.

quat_step advances a quaternion by a body-frame angular velocity that is
constant over dt. It is exact under that assumption and keeps q unit length.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103). Not NED.
  * Attitude is a unit quaternion q = [x, y, z, w], scalar-last, Hamilton
    product, body to world: v_world = R(q) @ v_body. No Euler angles.
  * Angular velocity w is in the body frame, rad/s. Time in s.
  * SI units.

Governing equations:
  Euler:  x_next = x + dt * f(x, u)
  RK4:    k1 = f(x, u)
          k2 = f(x + 0.5 * dt * k1, u)
          k3 = f(x + 0.5 * dt * k2, u)
          k4 = f(x + dt * k3, u)
          x_next = x + (dt / 6) * (k1 + 2 k2 + 2 k3 + k4)
  Attitude: q_next = normalize(q (x) Exp(w * dt)),
            the exact solution of q_dot = 0.5 * q (x) [w, 0] for constant w.
            Here w = [wx, wy, wz] is the body angular velocity (not the
            scalar part of q), and [w, 0] = [wx, wy, wz, 0] is its
            scalar-last pure quaternion.

Implementations of quat_step may use quat_multiply, quat_exp and
quat_normalize from hsr_ekf.quaternions. Every function returns NumPy
arrays of float and does not modify its inputs.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from typing import Callable


def euler_step(f: Callable, x, u, dt: float) -> np.ndarray:
    """One explicit (forward) Euler step of x_dot = f(x, u).

    x_next = x + dt * f(x, u)

    Local error is O(dt^2), global error O(dt). f is called once.

    Parameters
    ----------
    f : callable
        f(x, u) -> array-like of the same shape as x: the time derivative of
        x. It must not modify its arguments.
    x : array-like, shape (n,)
        State at the start of the step.
    u : any
        Input, passed unchanged to f and held constant over the step. May be
        None for a model without input.
    dt : float
        Step length, s. dt = 0 returns a copy of x; a negative dt steps
        backward in time.

    Returns
    -------
    np.ndarray, shape (n,)
        State after dt. A new array; x is not modified. Quaternion entries
        are NOT renormalized.
    """
    raise NotImplementedError("euler_step")


def rk4_step(f: Callable, x, u, dt: float) -> np.ndarray:
    """One classical fourth-order Runge-Kutta step of x_dot = f(x, u).

    k1 = f(x, u)
    k2 = f(x + 0.5 * dt * k1, u)
    k3 = f(x + 0.5 * dt * k2, u)
    k4 = f(x + dt * k3, u)
    x_next = x + (dt / 6) * (k1 + 2 k2 + 2 k3 + k4)

    Local error is O(dt^5), global error O(dt^4). f is called four times,
    each time with the same u (zero-order hold over the step).

    Parameters
    ----------
    f : callable
        f(x, u) -> array-like of the same shape as x: the time derivative of
        x. It must not modify its arguments. The intermediate states passed
        to f are plain vector sums, so a quaternion inside them is not unit
        length; f must accept that (for example by normalizing q itself
        before using it).
    x : array-like, shape (n,)
        State at the start of the step.
    u : any
        Input, passed unchanged to f. May be None for a model without input.
    dt : float
        Step length, s. dt = 0 returns a copy of x; a negative dt steps
        backward in time.

    Returns
    -------
    np.ndarray, shape (n,)
        State after dt. A new array; x is not modified. Quaternion entries
        are NOT renormalized.
    """
    raise NotImplementedError("rk4_step")


def quat_step(q, w, dt: float) -> np.ndarray:
    """Advance an attitude quaternion by a constant body-frame angular velocity.

    q_next = normalize(q (x) Exp(w * dt))

    Exp(w * dt) is the unit quaternion of a rotation by |w| * dt radians
    about the direction of w (quat_exp of the rotation vector w * dt). It
    multiplies on the RIGHT of q because w is in the body frame. This is the
    exact solution of q_dot = 0.5 * q (x) [w, 0], with [w, 0] = [wx, wy, wz, 0]
    the scalar-last pure quaternion of w, when w is constant over the step,
    for any size of |w| * dt, so it has no integration error of its own.

    Parameters
    ----------
    q : array-like, shape (4,)
        Attitude at the start of the step, body to world, scalar-last
        [x, y, z, w]. Unit norm expected (within about 1e-9).
    w : array-like, shape (3,)
        Angular velocity, body frame, rad/s, constant over the step.
    dt : float
        Step length, s. A negative dt rotates backward.

    Returns
    -------
    np.ndarray, shape (4,)
        Attitude after dt, unit norm, scalar-last. w = 0 or dt = 0 returns
        q normalized. A new array; q is not modified.

    Raises
    ------
    ValueError
        If q has zero norm.
    """
    raise NotImplementedError("quat_step")
