"""Process models: how the state moves forward in time between measurements.

A process model gives the error-state EKF three things for one time step of
length dt: the nominal state after the step (step), the error-state
transition matrix Phi (error_jacobian), and the discrete process-noise
covariance Q_d (noise). The filter core (hsr_ekf.ekf) only calls these
three methods, so the two models below are interchangeable:

  * ConstantVelocityModel: v and w stay constant; p and q follow the
    kinematics. Needs no vehicle parameters and ignores the control input.
  * HydroModel: v and w follow the rigid-body equation of motion
        M nu_dot + C(nu) nu + D(nu) nu + g(q) = tau
    (hsr_ekf.dynamics), driven by the body-frame wrench tau.

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
    q_true = q (x) Exp(dtheta).
  * nu = [v; w], shape (6,), body frame. tau = [force; torque], shape (6,),
    body frame, N and N*m.
  * SI units throughout (m, s, N, rad).

Continuous error-state Jacobian F (12 x 12), shared by both models, with
R = R(q) and [a]x = skew(a). Rows and columns are ordered [dp, dtheta, dv, dw]:

    dp_dot     = -R [v]x dtheta + R dv
    dtheta_dot = -[w]x dtheta + dw
    dv_dot     = (model specific)
    dw_dot     = (model specific)

The dp row comes from R_true v_true = R (I + [dtheta]x)(v + dv)
~= R v - R [v]x dtheta + R dv. The dtheta row is the standard result for a
body-frame (right) attitude error driven by a body-frame rate.

Discrete forms used by both models, to first order in dt:
    Phi = I + F dt                           (12, 12)
    Q_d = Q_c dt                             (12, 12)
with Q_c = diag(0, 0, 0, 0, 0, 0, a_x, a_y, a_z, b_x, b_y, b_z), where
a = accel_noise (m^2/s^3) and b = ang_accel_noise (rad^2/s^3) are the power
spectral densities of the white noise entering dv_dot and dw_dot, per body
axis (variance added per second, not standard deviations). Terms of order
dt^2 and higher are dropped, so p and theta receive noise only through Phi
on later steps.

Functions used by the bodies (import them from the sibling modules when
writing them): quaternions.quat_rotate, quat_normalize and quat_to_R;
kinematics.position_rate and quat_rate; integrate.rk4_step and quat_step;
rotations.skew; dynamics.mass_matrix and body_accel; jacobians.numerical_jacobian.
Only type hints are imported here, so this module imports without pulling in
the dynamics.
"""
from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .dynamics import VehicleParams


class ProcessModel:
    """Interface for a process model used by the error-state EKF.

    Subclasses implement all three methods. Within one predict call, the
    filter evaluates error_jacobian(x, u, dt) and noise(dt) at the state at
    the START of the step, then replaces the state with step(x, u, dt).

    A subclass must keep the three methods consistent: for small dt,
    error_jacobian(x, u, dt) must agree with
    jacobians.numerical_error_jacobian(self.step, x, u, dt) up to terms of
    order dt^2 (halving dt shrinks the difference by about 4x).

    None of the methods modifies its arguments.
    """

    def step(self, x, u, dt: float) -> np.ndarray:
        """Nominal state after ``dt`` seconds, with no process noise.

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state [p, q, v, w] at the start of the step; q unit norm.
        u : any
            Control input over the step (zero-order hold). Its meaning is
            model specific; see the subclass.
        dt : float
            Step length, s. dt = 0 returns x with q normalized.

        Returns
        -------
        np.ndarray, shape (13,)
            Full state after dt, with a unit-norm quaternion. A new array.

        Raises
        ------
        NotImplementedError
            Always, in this base class.
        """
        raise NotImplementedError("ProcessModel.step")

    def error_jacobian(self, x, u, dt: float) -> np.ndarray:
        """Error-state transition matrix Phi for one step from ``x``.

        To first order, an error dx at the start of the step becomes
        Phi @ dx at the end:
            step(x (+) dx, u, dt) (-) step(x, u, dt) ~= Phi @ dx.

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state at the start of the step (the linearization point).
        u : any
            Same control input that is passed to step.
        dt : float
            Step length, s. dt = 0 returns the 12 x 12 identity.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Phi, rows and columns ordered [dp, dtheta, dv, dw].

        Raises
        ------
        NotImplementedError
            Always, in this base class.
        """
        raise NotImplementedError("ProcessModel.error_jacobian")

    def noise(self, dt: float) -> np.ndarray:
        """Discrete process-noise covariance Q_d for one step of length ``dt``.

        Parameters
        ----------
        dt : float
            Step length, s, >= 0. dt = 0 returns a 12 x 12 zero matrix.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Symmetric positive semi-definite, ordered [dp, dtheta, dv, dw],
            in the squared units of each error-state entry.

        Raises
        ------
        NotImplementedError
            Always, in this base class.
        """
        raise NotImplementedError("ProcessModel.noise")


class ConstantVelocityModel(ProcessModel):
    """Constant body-frame velocity: v and w do not change; p and q follow them.

    Continuous model (u is ignored):
        p_dot = R(q) v
        q_dot = 0.5 * q (x) [w, 0]       ([w, 0] = [wx, wy, wz, 0], scalar-last)
        v_dot = 0 + white noise, density accel_noise
        w_dot = 0 + white noise, density ang_accel_noise

    This model needs no vehicle parameters, so it is the baseline to compare
    HydroModel against. It is close to what the stock robot_localization EKF
    does on the sub (no sensor fuses acceleration there and use_control is
    unset).
    """

    def __init__(self, accel_noise, ang_accel_noise):
        """Store the continuous white-noise densities on the body accelerations.

        Parameters
        ----------
        accel_noise : float or array-like, shape (3,)
            Power spectral density of the white noise on v_dot, per body axis
            [x, y, z], in (m/s^2)^2 / Hz = m^2/s^3. A float applies to all
            three axes. Over a step dt it adds accel_noise * dt (m/s)^2 of
            variance to v.
        ang_accel_noise : float or array-like, shape (3,)
            Power spectral density of the white noise on w_dot, per body axis,
            in (rad/s^2)^2 / Hz = rad^2/s^3. A float applies to all three axes.

        Attributes set
        --------------
        accel_noise : np.ndarray, shape (3,)
            Float copy of ``accel_noise``, broadcast to three axes.
        ang_accel_noise : np.ndarray, shape (3,)
            Float copy of ``ang_accel_noise``, broadcast to three axes.

        Raises
        ------
        ValueError
            If either density has any negative entry, or cannot be broadcast
            to shape (3,).
        """
        raise NotImplementedError("ConstantVelocityModel.__init__")

    def step(self, x, u, dt: float) -> np.ndarray:
        """Nominal state after ``dt`` with constant v and w.

        With q, v, w taken from x:
            q_next = quat_step(q, w, dt)            exact for constant w
            q_mid  = quat_step(q, w, 0.5 * dt)
            p_next = p + quat_rotate(q_mid, v) * dt  (midpoint rule, error O(dt^3))
            v_next = v
            w_next = w

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state [p, q, v, w] at the start of the step; q unit norm.
        u : any
            Ignored (may be None or a tau array), so the same control sequence
            can drive either model.
        dt : float
            Step length, s. dt = 0 returns x with q normalized; a negative dt
            steps backward.

        Returns
        -------
        np.ndarray, shape (13,)
            Full state after dt, unit-norm q. A new array; x is not modified.

        Raises
        ------
        ValueError
            If x does not have shape (13,).
        """
        raise NotImplementedError("ConstantVelocityModel.step")

    def error_jacobian(self, x, u, dt: float) -> np.ndarray:
        """Error-state transition matrix Phi = I + F dt for the constant-velocity model.

        With R = R(q) from x and [a]x = skew(a), F is

            rows \\ cols   dp     dtheta      dv     dw
            dp            0      -R [v]x     R      0
            dtheta        0      -[w]x       0      I
            dv            0      0           0      0
            dw            0      0           0      0

        so Phi is the 12 x 12 identity plus these blocks times dt:
            Phi[0:3, 3:6]  = -R @ skew(v) * dt
            Phi[0:3, 6:9]  =  R * dt
            Phi[3:6, 3:6]  =  I - skew(w) * dt
            Phi[3:6, 9:12] =  I * dt
        Everything else equals the identity. The exact attitude block is
        R(Exp(w dt))^T; I - skew(w) dt is its first-order form.

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state at the start of the step (the linearization point).
        u : any
            Ignored.
        dt : float
            Step length, s. dt = 0 returns the identity.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Phi, ordered [dp, dtheta, dv, dw]. Agrees with
            jacobians.numerical_error_jacobian(self.step, x, u, dt) up to
            terms of order dt^2.

        Raises
        ------
        ValueError
            If x does not have shape (13,).
        """
        raise NotImplementedError("ConstantVelocityModel.error_jacobian")

    def noise(self, dt: float) -> np.ndarray:
        """Discrete process noise Q_d = Q_c dt for the constant-velocity model.

        Q_d is zero except for two diagonal blocks:
            Q_d[6:9, 6:9]   = diag(accel_noise) * dt          (m/s)^2
            Q_d[9:12, 9:12] = diag(ang_accel_noise) * dt      (rad/s)^2

        Parameters
        ----------
        dt : float
            Step length, s, >= 0. dt = 0 returns a 12 x 12 zero matrix.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Diagonal, positive semi-definite, ordered [dp, dtheta, dv, dw].

        Raises
        ------
        ValueError
            If dt < 0 (a negative variance is not a covariance).
        """
        raise NotImplementedError("ConstantVelocityModel.noise")


class HydroModel(ProcessModel):
    """Hydrodynamic model: nu follows the equation of motion driven by tau.

    Continuous model, with u = tau (6,) in the body frame:
        p_dot  = R(q) v
        q_dot  = 0.5 * q (x) [w, 0]
        nu_dot = body_accel(params, q, nu, tau)
               = M^-1 (tau - C(nu) nu - D(nu) nu - g(q))
    plus white noise on v_dot and w_dot (densities accel_noise and
    ang_accel_noise) for unmodelled forces: currents, tether drag, and error
    in the fitted coefficients.

    tau must be the body-frame wrench in N and N*m, applied at the body
    origin. Converting a /wrench message into these units is the caller's
    job (the units of /wrench on the sub are not yet confirmed; they may be
    kgf).
    """

    def __init__(self, params: VehicleParams, accel_noise, ang_accel_noise):
        """Store the vehicle parameters and the process-noise densities.

        Parameters
        ----------
        params : VehicleParams
            Rigid-body, added-mass, damping and restoring parameters
            (hsr_ekf.dynamics), for example from dynamics.load_params.
            Stored by reference, not copied.
        accel_noise : float or array-like, shape (3,)
            Power spectral density of the white noise on v_dot, per body axis
            [x, y, z], in m^2/s^3. A float applies to all three axes.
        ang_accel_noise : float or array-like, shape (3,)
            Power spectral density of the white noise on w_dot, per body axis,
            in rad^2/s^3. A float applies to all three axes.

        Attributes set
        --------------
        params : VehicleParams
            The ``params`` argument.
        accel_noise : np.ndarray, shape (3,)
            Float copy of ``accel_noise``, broadcast to three axes.
        ang_accel_noise : np.ndarray, shape (3,)
            Float copy of ``ang_accel_noise``, broadcast to three axes.

        Raises
        ------
        ValueError
            If either density has any negative entry, or cannot be broadcast
            to shape (3,).
        """
        raise NotImplementedError("HydroModel.__init__")

    def step(self, x, u, dt: float) -> np.ndarray:
        """Nominal state after ``dt`` under the constant wrench ``u``, by RK4.

        Define the full-state derivative f(x, tau), shape (13,):
            qn = quat_normalize(x[3:7])     (RK4 stages pass a non-unit q)
            f[0:3]   = quat_rotate(qn, v)                   p_dot, world
            f[3:7]   = 0.5 * qn (x) [w, 0]                  q_dot
            f[7:13]  = body_accel(params, qn, [v; w], tau)  nu_dot, body
        Then
            x_next = integrate.rk4_step(f, x, tau, dt)
            x_next[3:7] = quat_normalize(x_next[3:7])
        tau is held constant over the step (zero-order hold).

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state [p, q, v, w] at the start of the step; q unit norm.
        u : array-like, shape (6,)
            tau = [X, Y, Z, K, M, N]: body-frame force (N) and torque (N*m).
        dt : float
            Step length, s. dt = 0 returns x with q normalized; a negative dt
            steps backward.

        Returns
        -------
        np.ndarray, shape (13,)
            Full state after dt, unit-norm q. A new array; x is not modified.
            Sign check: with q = identity, nu = 0, tau = 0, r_g = r_b = 0
            and B > W, a small step gives v[2] > 0 (the vehicle rises).

        Raises
        ------
        ValueError
            If x does not have shape (13,), or u is None or does not have
            shape (6,).
        """
        raise NotImplementedError("HydroModel.step")

    def error_jacobian(self, x, u, dt: float) -> np.ndarray:
        """Error-state transition matrix Phi = I + F dt for the hydrodynamic model.

        With R = R(q), e3_b = R^T e3 (world up seen in the body frame,
        e3 = [0, 0, 1]), and [a]x = skew(a), F is

            rows \\ cols   dp     dtheta      dv      dw
            dp            0      -R [v]x     R       0
            dtheta        0      -[w]x       0       I
            dv            0      A_th[0:3]   A[0:3, 0:3]   A[0:3, 3:6]
            dw            0      A_th[3:6]   A[3:6, 0:3]   A[3:6, 3:6]

        The dynamics blocks:
          * A = d nu_dot / d nu, shape (6, 6), at nu. Computed as
            jacobians.numerical_jacobian(lambda n: body_accel(params, q, n, tau), nu)
            (C(nu) nu and D(nu) nu have awkward closed-form derivatives).
            A depends only on nu: tau enters nu_dot additively and q only
            through g, so both drop out of the derivative.
          * A_th = d nu_dot / d dtheta = -M^-1 @ dg, shape (6, 3), from the
            restoring vector g(q) = [(W - B) e3_b; (W r_g - B r_b) x e3_b].
            With a right perturbation, e3_b_true = Exp(-dtheta) e3_b
            ~= e3_b + [e3_b]x dtheta, so
                dg = [ (W - B) * skew(e3_b) ;
                       skew(W r_g - B r_b) @ skew(e3_b) ]       shape (6, 3)
            M = dynamics.mass_matrix(params) is constant.
        tau and p do not enter F (no depth-dependent forces are modelled).

        Phi is the identity plus F * dt, block by block as above.

        Parameters
        ----------
        x : array-like, shape (13,)
            Full state at the start of the step (the linearization point).
        u : array-like, shape (6,)
            Same tau that is passed to step, N and N*m, body frame.
        dt : float
            Step length, s. dt = 0 returns the identity.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Phi, ordered [dp, dtheta, dv, dw]. Agrees with
            jacobians.numerical_error_jacobian(self.step, x, u, dt) up to
            terms of order dt^2.

        Raises
        ------
        ValueError
            If x does not have shape (13,), or u is None or does not have
            shape (6,).
        """
        raise NotImplementedError("HydroModel.error_jacobian")

    def noise(self, dt: float) -> np.ndarray:
        """Discrete process noise Q_d = Q_c dt for the hydrodynamic model.

        Same form as the constant-velocity model; only the meaning of the
        densities differs (unmodelled forces, not all acceleration):
            Q_d[6:9, 6:9]   = diag(accel_noise) * dt          (m/s)^2
            Q_d[9:12, 9:12] = diag(ang_accel_noise) * dt      (rad/s)^2
        and zero elsewhere. With a good model these densities should be much
        smaller than the constant-velocity model's.

        Parameters
        ----------
        dt : float
            Step length, s, >= 0. dt = 0 returns a 12 x 12 zero matrix.

        Returns
        -------
        np.ndarray, shape (12, 12)
            Diagonal, positive semi-definite, ordered [dp, dtheta, dv, dw].

        Raises
        ------
        ValueError
            If dt < 0.
        """
        raise NotImplementedError("HydroModel.noise")
