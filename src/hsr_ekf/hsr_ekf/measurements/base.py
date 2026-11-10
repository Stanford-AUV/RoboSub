"""Measurement interface shared by every sensor model of the error-state EKF.

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103), not NED.
  * Full state x, shape (13,): p = x[0:3] (position, world, m),
    q = x[3:7] (attitude, body to world, scalar-last [x, y, z, w]),
    v = x[7:10] (linear velocity, body, m/s), w = x[10:13] (angular
    velocity, body, rad/s). No Euler angles.
  * Error state dx, shape (12,): dx[0:3] = dp, dx[3:6] = dtheta (rotation
    vector, body frame, rad), dx[6:9] = dv, dx[9:12] = dw. The attitude
    error is applied on the right: q_true = q (x) Exp(dtheta). The slices
    and state_plus / state_minus live in hsr_ekf.state.
  * SI units.

A measurement model tells the filter core (hsr_ekf.ekf) three things about
one sensor:
  1. predict(x): the noise-free reading the sensor would give if the state
     were exactly x.
  2. residual(z, z_pred): the difference between a real reading z and that
     prediction, as a plain vector of length dim.
  3. jacobian(x): the linear map H from a small error-state step dx to the
     change in the residual.

The core reads only these three methods and the attributes dim, R and gate.
It never imports a specific sensor module, so a sensor is added as a new
module that subclasses Measurement, and the core does not change.
"""
from __future__ import annotations

from typing import Optional

import numpy as np


class Measurement:
    """Interface for one sensor's measurement model in the error-state EKF.

    A subclass sets ``dim``, ``R`` and ``gate`` in its ``__init__`` and
    implements ``predict`` and ``jacobian``. ``residual`` has a default
    (plain subtraction); a subclass overrides it only when its reading is
    not an ordinary vector, for example a quaternion.

    Attributes
    ----------
    dim : int
        Length of the residual vector. It is also the number of rows of H
        and the size of R. It can differ from the length of the raw reading
        z: an orientation quaternion z has 4 entries, but its residual is a
        3-vector rotation, so dim is 3.
    R : np.ndarray, shape (dim, dim)
        Measurement noise covariance of the residual, in residual units
        squared (for example m^2 or rad^2). Symmetric positive definite.
        Fixed at construction.
    gate : float or None
        Chi-squared threshold on the normalized innovation squared,
        NIS = y^T S^-1 y with y the residual and S = H P H^T + R. The filter
        skips the update (rejects the reading) when NIS > gate. None turns
        gating off, so every reading is used. A threshold is normally a
        chi-squared quantile with dim degrees of freedom: 99% gives 6.635
        (dim 1) and 11.345 (dim 3); 95% gives 3.841 (dim 1) and 7.815
        (dim 3).

    Notes
    -----
    How the filter uses a measurement, given the nominal state x, the
    error covariance P (12, 12) and a reading z:

        z_pred = predict(x)
        y      = residual(z, z_pred)          shape (dim,)
        H      = jacobian(x)                  shape (dim, 12)
        S      = H P H^T + R                  shape (dim, dim)
        NIS    = y^T S^-1 y                   reject if gate is not None and NIS > gate
        K      = P H^T S^-1                   shape (12, dim)
        dx     = K y                          then x <- state_plus(x, dx)

    Jacobian convention. H is taken with respect to the 12-dimensional
    ERROR state, not the 13-dimensional full state, and it must agree with
    the residual's sign:

        residual(predict(state_plus(x, dx)), predict(x)) ~= H @ dx   for small dx

    so if the true state is x (+) dx and the reading is noise-free, the
    residual is H dx to first order. Columns follow the error-state layout
    [dp (0:3), dtheta (3:6), dv (6:9), dw (9:12)]; the attitude columns are
    derivatives with respect to the body-frame rotation vector dtheta, not
    with respect to quaternion components. A subclass can check its H with
    hsr_ekf.jacobians.numerical_jacobian applied to
    dx -> residual(predict(state_plus(x, dx)), predict(x)) at dx = zeros(12).

    Adding a sensor:
      1. Write a new module in measurements/ with a subclass of Measurement.
      2. In ``__init__`` set ``dim``, ``R`` (dim, dim) and ``gate``.
      3. Implement ``predict`` and ``jacobian``; override ``residual`` only
         if the reading is not an ordinary vector.
      4. Check ``jacobian`` against the numerical Jacobian above.
      5. Re-export the class from measurements/__init__.py.
    """

    dim: int
    R: np.ndarray
    gate: Optional[float]

    def predict(self, x: np.ndarray) -> np.ndarray:
        """Noise-free reading the sensor would report if the state were exactly ``x``.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w] (see the module docstring).
            Not modified.

        Returns
        -------
        np.ndarray
            z_pred, with the same shape, frame and units as this sensor's
            raw reading z. This is (dim,) for a vector reading, but not
            always: an orientation reading is a (4,) quaternion while dim
            is 3.

        Raises
        ------
        NotImplementedError
            Always, in this base class. Every sensor overrides it.
        """
        raise NotImplementedError("Measurement.predict")

    def residual(self, z, z_pred) -> np.ndarray:
        """Residual (innovation) y between a real reading and the prediction.

        Default: y = z - z_pred, element by element, as a float array.
        Subclasses whose readings are not ordinary vectors (a quaternion)
        override this; the result must still be a (dim,) vector whose
        first-order behaviour matches ``jacobian`` (see the class Notes).

        The order of the arguments sets the sign. The filter always passes
        the real reading first and predict(x) second.

        Parameters
        ----------
        z : array-like, shape (dim,)
            The real reading, in the sensor's frame and units.
        z_pred : array-like, shape (dim,)
            The prediction from ``predict``.

        Returns
        -------
        np.ndarray, shape (dim,)
            z - z_pred, dtype float.

        Raises
        ------
        ValueError
            If ``z`` or ``z_pred`` does not have shape (dim,). A scalar
            reading must be passed as a length-1 array, e.g. np.array([z]).
        """
        raise NotImplementedError("Measurement.residual")

    def jacobian(self, x: np.ndarray) -> np.ndarray:
        """Jacobian H of the residual with respect to the error state, at ``x``.

        Parameters
        ----------
        x : np.ndarray, shape (13,)
            Full nominal state [p, q, v, w]. Not modified.

        Returns
        -------
        np.ndarray, shape (dim, 12)
            H, with columns in the error-state order [dp, dtheta, dv, dw],
            defined by residual(predict(state_plus(x, dx)), predict(x))
            ~= H @ dx for small dx (see the class Notes).

        Raises
        ------
        NotImplementedError
            Always, in this base class. Every sensor overrides it.
        """
        raise NotImplementedError("Measurement.jacobian")
