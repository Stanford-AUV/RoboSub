"""Error-state EKF core: predict with a process model, update with any measurement.

The filter keeps a nominal full state x (13,) and the covariance P (12, 12)
of a small error state dx around it. One cycle:

  predict(u, dt):  Phi = process.error_jacobian(x, u, dt)
                   Q_d = process.noise(dt)
                   x   = process.step(x, u, dt)
                   P   = Phi P Phi^T + Q_d
  update(m, z):    y   = m.residual(z, m.predict(x))      innovation
                   H   = m.jacobian(x)                     d z / d dx
                   S   = H P H^T + m.R
                   gate on NIS = y^T S^-1 y
                   K   = P H^T S^-1,  dx = K y
                   x   = x (+) dx                          inject
                   P   = Joseph form, then the reset G P G^T

The core is sensor-agnostic. It uses only the Measurement interface
(hsr_ekf.measurements.base: dim, R, gate, predict, residual, jacobian) and
the ProcessModel interface (hsr_ekf.process), and it imports no specific
sensor module. A new sensor is a new module under measurements/; this file
does not change.

The filter does not track time stamps. The caller calls predict(u, dt) up
to the time of each measurement, then update; out-of-order measurements are
not supported.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * No Euler angles. Attitude is a unit quaternion stored scalar-last,
    q = [x, y, z, w], Hamilton product, body to world:
    v_world = R(q) @ v_body.
  * Full state x, shape (13,): x[0:3] = p (world, m), x[3:7] = q
    (body -> world), x[7:10] = v (body, m/s), x[10:13] = w (body, rad/s).
  * Error state dx, shape (12,): dx[0:3] = dp (world, m), dx[3:6] = dtheta
    (body-frame rotation vector, rad), dx[6:9] = dv (body, m/s),
    dx[9:12] = dw (body, rad/s). Right perturbation:
    q_true = q (x) Exp(dtheta). Index with state.DP, DTH, DV, DW.
  * Controls u = tau = [force; torque], shape (6,), body frame, N and N*m
    (ignored by the constant-velocity model).
  * SI units throughout (m, s, N, rad).

Functions used by the bodies (import them from the sibling modules when
writing them): state.state_plus, state.DTH; quaternions.quat_normalize;
rotations.skew; kf.nis. Only type hints are imported here, so this module
imports without pulling in any process or sensor module.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .measurements.base import Measurement
    from .process import ProcessModel


@dataclass
class UpdateResult:
    """Outcome of one ErrorStateEKF.update call.

    Attributes
    ----------
    accepted : bool
        True if the measurement was applied. False if the gate rejected it,
        in which case the filter state and covariance are unchanged.
    residual : np.ndarray, shape (dim,)
        Innovation y = measurement.residual(z, z_pred), computed with the
        state BEFORE the update. For the attitude measurement this is a
        body-frame rotation vector (3,), in rad.
    S : np.ndarray, shape (dim, dim)
        Innovation covariance H P H^T + R, before the update.
    nis : float
        Normalized innovation squared y^T S^-1 y (>= 0). For a consistent
        filter it follows a chi-squared distribution with ``dim`` degrees of
        freedom.
    """

    accepted: bool
    residual: np.ndarray
    S: np.ndarray
    nis: float


class ErrorStateEKF:
    """Quaternion error-state Extended Kalman Filter.

    Holds the nominal state x (13,) and the error-state covariance P
    (12, 12). Attitude errors are 3-vectors in the body frame, so P never
    carries the quaternion's redundant fourth dimension.

    Use: build with a process model and an initial estimate, then call
    predict(u, dt) and update(measurement, z) in time order. Read the
    estimate through the ``state`` and ``covariance`` properties.
    """

    def __init__(self, process: ProcessModel, x0, P0):
        """Build the filter from a process model and an initial estimate.

        Parameters
        ----------
        process : ProcessModel
            Any process model (hsr_ekf.process). Stored by reference.
        x0 : array-like, shape (13,)
            Initial full state [p, q, v, w]. The quaternion is normalized on
            entry (q and -q are the same attitude; either is accepted).
        P0 : array-like, shape (12, 12)
            Initial error-state covariance, ordered [dp, dtheta, dv, dw], in
            the squared units of each entry (m^2, rad^2, (m/s)^2, (rad/s)^2).
            Must be symmetric positive semi-definite; it is stored as
            0.5 * (P0 + P0^T).

        Attributes set
        --------------
        process : ProcessModel
            The ``process`` argument.
        _x : np.ndarray, shape (13,)
            Float copy of x0 with q normalized. Never aliases the caller's array.
        _P : np.ndarray, shape (12, 12)
            Float copy of P0, symmetrized.

        Raises
        ------
        ValueError
            If x0 does not have shape (13,), P0 does not have shape (12, 12),
            or the quaternion in x0 has zero norm.
        """
        raise NotImplementedError("ErrorStateEKF.__init__")

    def predict(self, u, dt: float) -> None:
        """Propagate the estimate forward by ``dt`` seconds.

        In this order (Phi and Q_d use the state at the START of the step):
            Phi = process.error_jacobian(x, u, dt)       (12, 12)
            Q_d = process.noise(dt)                      (12, 12)
            x   = process.step(x, u, dt)                 (13,)
            P   = Phi P Phi^T + Q_d
            P   = 0.5 * (P + P^T)                        keep P symmetric

        Parameters
        ----------
        u : any
            Control input over the step, passed unchanged to the process
            model: tau (6,) in N and N*m, body frame, for HydroModel; ignored
            (may be None) by ConstantVelocityModel.
        dt : float
            Step length, s, >= 0. dt = 0 leaves P unchanged and x unchanged
            up to the rounding of renormalizing q.

        Returns
        -------
        None
            The filter is updated in place.

        Raises
        ------
        ValueError
            If dt < 0. Errors raised by the process model (for example a
            wrong u shape) propagate. All three process calls run before x
            or P is reassigned, so after any error the filter is unchanged.
        """
        raise NotImplementedError("ErrorStateEKF.predict")

    def update(self, measurement: Measurement, z) -> UpdateResult:
        """Correct the estimate with one measurement ``z`` from ``measurement``.

        Steps, with x and P the current estimate and dim = measurement.dim:
          1. z_pred = measurement.predict(x)
          2. y = measurement.residual(z, z_pred)          shape (dim,)
          3. H = measurement.jacobian(x)                  shape (dim, 12)
          4. S = H P H^T + measurement.R                  shape (dim, dim)
          5. nis = y^T S^-1 y                             (kf.nis; solve, do not invert)
          6. Gate: if measurement.gate is not None and nis > measurement.gate,
             return UpdateResult(False, y, S, nis) and change nothing.
          7. K = P H^T S^-1                               shape (12, dim)
             Without an explicit inverse: K = np.linalg.solve(S, H @ P).T,
             which equals P H^T S^-1 because P and S are symmetric.
          8. dx = K y                                     shape (12,)
          9. Inject: x = state_plus(x, dx). p, v, w add; q = q (x) Exp(dtheta),
             normalized.
         10. Joseph form, valid for any K and numerically safer than (I - K H) P:
             P = (I - K H) P (I - K H)^T + K R K^T
         11. Reset. The error state is now measured from the new nominal x,
             so P is re-expressed in that frame:
                 P = G P G^T
             G is the 12 x 12 identity except the attitude block
                 G[3:6, 3:6] = I - 0.5 * skew(dtheta),   dtheta = dx[3:6]
             (first-order form of the right Jacobian of SO(3) at dtheta, for
             a body-frame attitude error). Taking G[3:6, 3:6] = I, which
             makes this step a no-op, is the usual simplification; it
             neglects a term of order |dtheta| * P[3:6, 3:6] and is
             acceptable when corrections are small, which the gate helps
             ensure. This method uses the I - 0.5 * skew(dtheta) form.
         12. P = 0.5 * (P + P^T); return UpdateResult(True, y, S, nis).

        Parameters
        ----------
        measurement : Measurement
            Any object implementing the Measurement interface
            (hsr_ekf.measurements.base): attributes dim, R (dim, dim) and
            gate (float or None); methods predict, residual and jacobian.
        z : array-like
            The reading, in the form ``measurement`` expects (for example
            shape (1,) for depth, (3,) for the DVL or gyro, (4,) for the IMU
            quaternion). Not modified.

        Returns
        -------
        UpdateResult
            accepted, the pre-update residual y, S and the NIS. When rejected
            by the gate, x and P are exactly as before the call.

        Raises
        ------
        ValueError
            If y does not have shape (dim,), H does not have shape (dim, 12),
            or measurement.R does not have shape (dim, dim). Raised before x
            or P is modified.
        numpy.linalg.LinAlgError
            If S is singular (for example R = 0 with no uncertainty in the
            measured direction). Raised before x or P is modified.
        """
        raise NotImplementedError("ErrorStateEKF.update")

    @property
    def state(self) -> np.ndarray:
        """Current nominal full state [p, q, v, w], shape (13,), unit-norm q.

        Returns a copy, so changing the returned array does not change the
        filter. Read-only: there is no setter.
        """
        raise NotImplementedError("ErrorStateEKF.state")

    @property
    def covariance(self) -> np.ndarray:
        """Current error-state covariance P, shape (12, 12), ordered [dp, dtheta, dv, dw].

        Symmetric positive semi-definite, in the squared units of each entry.
        Returns a copy, so changing the returned array does not change the
        filter. Read-only: there is no setter.
        """
        raise NotImplementedError("ErrorStateEKF.covariance")
