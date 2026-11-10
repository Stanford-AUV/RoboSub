"""Jacobians by finite differences, used to check analytic Jacobians.

numerical_jacobian differentiates any plain vector function. It must not be
used directly on a full state that contains a quaternion: nudging one of the
four quaternion entries leaves the unit sphere and gives a 4-column block
that does not match the 3-entry attitude error.

numerical_error_jacobian differentiates a nominal step function through the
error state instead: it perturbs with state_plus and measures with
state_minus, so the attitude is always perturbed by a true rotation. Its
result is the discrete error-state transition matrix Phi that
ProcessModel.error_jacobian must return, which makes it the reference for
testing an analytic Phi.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103). Not NED.
  * Attitude is a unit quaternion q = [x, y, z, w], scalar-last, Hamilton
    product, body to world: v_world = R(q) @ v_body. No Euler angles.
  * Full state x (13,) = [p, q, v, w]; error state dx (12,) =
    [dp, dtheta, dv, dw] with right (body-frame) attitude perturbation
    q_true = q (x) Exp(dtheta). See hsr_ekf.state.
  * SI units; the step h is in the units of the perturbed entry
    (m, rad, m/s, rad/s).

Governing equations (e_j is the j-th standard basis vector):
  numerical_jacobian:
    J[:, j] = (fun(x + h e_j) - fun(x - h e_j)) / (2 h)
  numerical_error_jacobian, with x_next = step(x, u, dt):
    Phi[:, j] = ( state_minus(step(state_plus(x,  h e_j), u, dt), x_next)
                - state_minus(step(state_plus(x, -h e_j), u, dt), x_next) ) / (2 h)

Implementations of numerical_error_jacobian may use state_plus, state_minus
and ERROR_DIM from hsr_ekf.state. Every function returns NumPy arrays of
float and does not modify its inputs.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from typing import Callable


def numerical_jacobian(fun: Callable, x, h: float = 1e-6) -> np.ndarray:
    """Jacobian of a vector function by central differences.

    J[:, j] = (fun(x + h e_j) - fun(x - h e_j)) / (2 h),   j = 0 .. n-1

    Truncation error is O(h^2) per entry; rounding error grows like
    eps / h, so h = 1e-6 suits functions of order-one size in double
    precision. fun is called 2 n times; fun(x) itself is not needed.

    Parameters
    ----------
    fun : callable
        fun(x) -> array-like of shape (m,). A scalar return value is treated
        as shape (1,). It must not modify its argument.
    x : array-like, shape (n,)
        Point at which to differentiate. Plain real entries only (see the
        module docstring about quaternions). Converted to a float array
        before perturbing, so an integer input such as [1, 2] does not
        truncate the step h to zero.
    h : float, optional
        Finite-difference step, same units as x, > 0. Default 1e-6.

    Returns
    -------
    np.ndarray, shape (m, n)
        J[i, j] = d fun_i / d x_j at x. x is not modified (each perturbed
        point is a copy).

    Raises
    ------
    ValueError
        If h <= 0.
    """
    raise NotImplementedError("numerical_jacobian")


def numerical_error_jacobian(step: Callable, x, u, dt: float, h: float = 1e-6) -> np.ndarray:
    """Error-state transition matrix Phi of a nominal step, by central differences.

    With x_next = step(x, u, dt), column j is

      Phi[:, j] = ( state_minus(step(state_plus(x,  h e_j), u, dt), x_next)
                  - state_minus(step(state_plus(x, -h e_j), u, dt), x_next) ) / (2 h)

    so that, to first order, an error dx at the start of the step becomes
    Phi @ dx at the end:
      step(x (+) dx, u, dt) (-) step(x, u, dt) ~= Phi @ dx.

    Both perturbed results are measured from the same nominal x_next, which
    is the frame the filter's error state uses after predict. step is called
    2 * 12 + 1 = 25 times. Checks: for dt = 0 with a step that returns its
    input, Phi is the 12 x 12 identity up to rounding; for small dt,
    Phi ~= I + F dt with F the continuous error-state Jacobian.

    Parameters
    ----------
    step : callable
        step(x, u, dt) -> array-like, shape (13,): the nominal state after dt,
        for example ProcessModel.step. Its output quaternion must have unit
        norm. It must not modify its arguments.
    x : array-like, shape (13,)
        Full state [p, q, v, w] at the start of the step; q unit norm.
    u : any
        Input passed unchanged to step (tau (6,) in N and N*m for the
        hydrodynamic model; ignored by the constant-velocity model).
    dt : float
        Step length, s.
    h : float, optional
        Finite-difference step applied to each error-state entry, in that
        entry's units (m, rad, m/s, rad/s), > 0. Default 1e-6.

    Returns
    -------
    np.ndarray, shape (12, 12)
        Phi[i, j] = d dx_next_i / d dx_j, rows and columns ordered
        [dp, dtheta, dv, dw]. x is not modified.

    Raises
    ------
    ValueError
        If h <= 0 or x does not have shape (13,).
    """
    raise NotImplementedError("numerical_error_jacobian")
