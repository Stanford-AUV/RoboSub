"""Linear Kalman filter: predict, update, and the normalized innovation squared.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * Attitude elsewhere in the package is a scalar-last quaternion
    q = [x, y, z, w], body to world. This module never sees one: it works on
    plain vectors only, so every state here lives in an ordinary vector space
    where ``x + dx`` and ``z - H x`` are meaningful.
  * SI units inside the package (m, s, N, rad).

These functions are the textbook linear Kalman filter. The error-state EKF
(hsr_ekf.ekf) runs the same covariance algebra on its 12-dimensional error
state, but it handles the quaternion part of the nominal state itself
(state.state_plus to inject the correction, Measurement.residual to form the
innovation); do not pass a 13-element state with a quaternion in it to these
functions.

Shapes: n is the state dimension and m the measurement dimension. A scalar
measurement is passed as z of shape (1,), H of shape (1, n), R of shape (1, 1).
Inputs are converted with np.asarray(..., dtype=float). Shapes are checked
explicitly before any arithmetic, because NumPy broadcasting would otherwise
silently accept wrong shapes (for example a Q of shape (n,), a scalar R, or
a z of shape (1,) when m = 3) and return a wrong answer instead of failing.

No function here modifies its arguments in place: each returns new float
arrays.
"""
from __future__ import annotations

import numpy as np


def kf_predict(x, P, Phi, Q) -> tuple[np.ndarray, np.ndarray]:
    """Propagate a state estimate and its covariance one step through a linear model.

    Model: x_k+1 = Phi x_k + n_k, with n_k zero-mean white noise of covariance Q.

        x_pred = Phi @ x
        P_pred = Phi @ P @ Phi.T + Q

    Parameters
    ----------
    x : array-like, shape (n,)
        State estimate at the current step.
    P : array-like, shape (n, n)
        Covariance of ``x``. Symmetric positive semi-definite.
    Phi : array-like, shape (n, n)
        State transition matrix over the step (discrete time, already
        includes dt; for a continuous model x_dot = F x, Phi = expm(F dt),
        to first order I + F dt).
    Q : array-like, shape (n, n)
        Discrete process-noise covariance added over the step. Symmetric
        positive semi-definite.

    Returns
    -------
    (x_pred, P_pred) : (np.ndarray of shape (n,), np.ndarray of shape (n, n))
        ``P_pred`` is returned symmetrized as 0.5 * (P_pred + P_pred.T) to
        remove rounding asymmetry.

    Raises
    ------
    ValueError
        Unless ``x`` is 1-D with length n and ``P``, ``Phi`` and ``Q`` are
        all exactly (n, n). Checked explicitly; do not rely on broadcasting.
    """
    raise NotImplementedError("kf_predict")


def kf_update(x, P, z, H, R) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Correct a state estimate with one linear measurement z = H x + r.

    The measurement noise r is zero-mean white with covariance R. The steps:

        y = z - H @ x                                  (innovation, shape (m,))
        S = H @ P @ H.T + R                            (innovation covariance)
        K = P @ H.T @ S^-1                             (gain, shape (n, m))
        x_new = x + K @ y
        P_new = (I - K H) @ P @ (I - K H).T + K @ R @ K.T    (Joseph form)

    The Joseph form keeps P_new symmetric positive semi-definite even when K
    is not exactly optimal or rounding is present; the short form (I - K H) P
    can lose both properties. Compute K by solving the linear system
    S K.T = H P.T (np.linalg.solve) rather than forming S^-1 explicitly; this
    is the transpose of K S = P H.T and relies on S being symmetric.

    Parameters
    ----------
    x : array-like, shape (n,)
        Predicted state.
    P : array-like, shape (n, n)
        Covariance of ``x``. Symmetric positive semi-definite.
    z : array-like, shape (m,)
        Measurement vector, in the same units and order as the rows of ``H``.
    H : array-like, shape (m, n)
        Measurement matrix: the noise-free measurement of state x is H @ x.
    R : array-like, shape (m, m)
        Measurement-noise covariance. Symmetric positive definite (needed so
        that S is invertible).

    Returns
    -------
    (x_new, P_new, y, S) : (np.ndarray of shape (n,), np.ndarray of shape (n, n),
                            np.ndarray of shape (m,), np.ndarray of shape (m, m))
        x_new : corrected state.
        P_new : corrected covariance from the Joseph form, returned
            symmetrized as 0.5 * (P_new + P_new.T).
        y : the innovation z - H @ x, computed with the predicted (input)
            ``x``.
        S : the innovation covariance H P H^T + R. ``y`` and ``S`` are
            returned so the caller can compute nis(y, S) for gating and
            consistency checks. This function never gates: it always applies
            the update.

    Raises
    ------
    ValueError
        Unless ``x`` is 1-D with length n, ``P`` is (n, n), ``z`` is 1-D with
        length m (a 0-d scalar z is rejected), ``H`` is (m, n) and ``R`` is
        exactly (m, m). Checked explicitly; do not rely on broadcasting.
    numpy.linalg.LinAlgError
        If S is singular.
    """
    raise NotImplementedError("kf_update")


def nis(y, S) -> float:
    """Normalized innovation squared: y^T S^-1 y.

    For a consistent filter (correct models and noise covariances) the NIS of
    an m-dimensional innovation follows a chi-squared distribution with m
    degrees of freedom, so its mean is m. Comparing it with a chi-squared
    quantile gives an outlier gate: for example 3.84 (m = 1) or 7.81 (m = 3)
    rejects about 5% of good measurements.

    Compute it as y @ np.linalg.solve(S, y), not with an explicit inverse.

    Parameters
    ----------
    y : array-like, shape (m,)
        Innovation (measurement residual).
    S : array-like, shape (m, m)
        Innovation covariance of ``y``. Symmetric positive definite.

    Returns
    -------
    float
        A Python float, >= 0 for a positive definite S; 0.0 when y is zero.

    Raises
    ------
    ValueError
        Unless ``y`` is 1-D with length m and ``S`` is exactly (m, m).
    numpy.linalg.LinAlgError
        If S is singular.
    """
    raise NotImplementedError("nis")
