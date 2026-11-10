"""Dynamics: the hydrodynamic equation of motion of the vehicle in its body frame.

Equation of motion (Fossen's vector form, every term in the body frame):

    M nu_dot + C(nu) nu + D(nu) nu + g(q) = tau

    nu_dot = M^-1 (tau - C(nu) nu - D(nu) nu - g(q))

  * nu = [v; w], shape (6,): linear velocity v (m/s) and angular velocity w
    (rad/s) of ``base_link``, body frame.
  * tau = [force; torque], shape (6,): applied thruster force (N) and torque
    (N m) about the body origin, body frame.
  * M = M_RB + M_A: rigid-body plus added mass, (6, 6), constant.
  * C(nu) = C_RB(nu) + C_A(nu): Coriolis and centripetal terms, (6, 6).
  * D(nu): linear plus quadratic damping, (6, 6).
  * g(q): gravity and buoyancy (restoring) forces and moments, (6,).

Conventions (see also hsr_ekf/__init__.py):
  * Frames: world ``odom`` and body ``base_link``, x forward, y left, z up
    (ROS REP-103). Not NED: Fossen's tables use z down, so the restoring
    vector g has different signs here (see restoring). Diagonal added-mass
    and damping coefficients carry over from NED tables unchanged, because
    flipping the y and z axes leaves a diagonal matrix the same. The y and z
    components of r_g and r_b, and the products of inertia Ixy and Ixz, do
    change sign; Iyz does not.
  * Attitude is a unit quaternion q = [x, y, z, w], scalar-last, Hamilton
    product, body to world: v_world = R(q) @ v_body. No Euler angles.
  * Error-state and full-state layouts live in hsr_ekf.state; this module
    sees only q, nu and tau.
  * SI units throughout (m, s, kg, N, rad). Thrust commands on the /wrench
    topic may not be in N (the thrust limits in hardware/utils/thrusters.py
    are unlabelled, and their size matches T200 datasheet values in kgf);
    convert to N and N m before calling body_accel.
  * In this module the argument named ``p`` is a VehicleParams, not the
    position.

Sign conventions for the hydrodynamic coefficients follow Fossen: added mass
(X_udot, ...) and damping (X_u, X_u|u|, ...) coefficients are NEGATIVE
numbers, and the matrices put a minus sign in front of them, so M_A and D(nu)
come out positive (semi-)definite.

Coefficient names use SNAME notation, in which nu = [u, v, w, p, q, r]: u,
v, w are the linear velocity components along body x, y, z, and p, q, r the
angular velocity components about body x, y, z; X, Y, Z name forces and K,
M, N moments about x, y, z. These letters have that meaning ONLY inside a
coefficient name (X_u, M_qdot, Z_w|w|, ...). Everywhere else in this package
v and w are the 3-vectors nu[0:3] and nu[3:6], q is the attitude quaternion,
p (in this module) is the VehicleParams, and M is the 6x6 mass matrix.

Implementations may use skew from hsr_ekf.rotations and quat_to_R or
quat_rotate from hsr_ekf.quaternions. Every function returns NumPy arrays
of float and does not modify its inputs. load_params is the only function
that reads a file.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import yaml


@dataclass
class VehicleParams:
    """Physical parameters of the vehicle used by the hydrodynamic model.

    All fields are in SI units and the body frame ``base_link`` (x forward,
    y left, z up). Array fields are float NumPy arrays of the shapes below;
    load_params builds them from YAML and checks shapes and signs. This class
    does not validate anything itself, so code that constructs it directly
    (tests, simulations) must pass correctly shaped float arrays.

    Attributes
    ----------
    mass : float
        Dry mass of the vehicle, kg. Must be > 0.
    inertia : np.ndarray, shape (3, 3)
        Rigid-body inertia tensor about the body ORIGIN (not the centre of
        gravity), body axes, kg m^2. Symmetric positive definite. The array
        holds the matrix exactly as it is used in M, so the off-diagonal
        entries already include the minus sign: inertia[0, 1] = -Ixy, where
        Ixy = integral of x y dm (likewise [0, 2] = -Ixz, [1, 2] = -Iyz). If
        it was measured about the centre of gravity, I_CG, convert with the
        parallel-axis theorem: I_O = I_CG - mass * skew(r_g) @ skew(r_g).
    r_g : np.ndarray, shape (3,)
        Centre of gravity relative to the body origin, body frame, m.
        Zero if ``base_link`` is at the centre of gravity.
    r_b : np.ndarray, shape (3,)
        Centre of buoyancy relative to the body origin, body frame, m.
        For passive righting stability r_b lies above r_g (larger z).
    weight : float
        Weight W, N. Normally mass * 9.80665. Must be > 0.
    buoyancy : float
        Buoyancy B, N: water density * 9.80665 * displaced volume. Must be
        > 0. B > W means the vehicle floats up with zero thrust.
    added_mass : np.ndarray, shape (6,)
        Diagonal added-mass coefficients [X_udot, Y_vdot, Z_wdot, K_pdot,
        M_qdot, N_rdot], kg for the first three, kg m^2 for the last three.
        Fossen sign: each entry <= 0.
    linear_drag : np.ndarray, shape (6,)
        Diagonal linear damping coefficients [X_u, Y_v, Z_w, K_p, M_q, N_r],
        N s/m for the first three, N m s/rad for the last three. Each entry
        <= 0.
    quadratic_drag : np.ndarray, shape (6,)
        Diagonal quadratic damping coefficients [X_u|u|, Y_v|v|, Z_w|w|,
        K_p|p|, M_q|q|, N_r|r|], N s^2/m^2 for the first three,
        N m s^2/rad^2 for the last three. Each entry <= 0.
    """

    mass: float
    inertia: np.ndarray
    r_g: np.ndarray
    r_b: np.ndarray
    weight: float
    buoyancy: float
    added_mass: np.ndarray
    linear_drag: np.ndarray
    quadratic_drag: np.ndarray


def load_params(path) -> VehicleParams:
    """Read vehicle parameters from a YAML file and return a VehicleParams.

    The model file is normally ``config/vehicle_model.yaml`` in the
    hsr_ekf folder (src/hsr_ekf/config/vehicle_model.yaml),
    whose values are placeholders until the coefficients are fitted. It is
    a flat YAML mapping, all values in SI units (not kgf), body frame
    ``base_link`` (x forward, y left, z up)::

        mass: <float>                  # kg, > 0
        inertia:                       # kg m^2, about the body origin
          - [I11, I12, I13]            # the 3x3 matrix row by row, as used
          - [I12, I22, I23]            # in M: symmetric, off-diagonal
          - [I13, I23, I33]            # entries include the minus sign
        r_g: [x, y, z]                 # m, centre of gravity
        r_b: [x, y, z]                 # m, centre of buoyancy
        weight: <float>                # N, > 0
        buoyancy: <float>              # N, > 0
        added_mass: [X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot]   # <= 0
        linear_drag: [X_u, Y_v, Z_w, K_p, M_q, N_r]                    # <= 0
        quadratic_drag: [X_uu, Y_vv, Z_ww, K_pp, M_qq, N_rr]           # <= 0

    In quadratic_drag, X_uu stands for X_u|u| and so on, in the order of
    VehicleParams.quadratic_drag. Field meanings and units are those of
    VehicleParams. Keys not listed above are ignored. Read the file with
    yaml.safe_load (never yaml.load).

    Parameters
    ----------
    path : str or os.PathLike
        Path to the YAML file.

    Returns
    -------
    VehicleParams
        mass, weight and buoyancy as Python floats; inertia as a float array
        of shape (3, 3); r_g and r_b of shape (3,); added_mass, linear_drag
        and quadratic_drag of shape (6,).

    Raises
    ------
    FileNotFoundError
        If ``path`` does not exist.
    KeyError
        If any of the nine keys above is missing; the message names the key.
    ValueError
        If the file is empty or not a YAML mapping, an array has the wrong
        shape, mass, weight or buoyancy is not > 0, or any entry of
        added_mass, linear_drag or quadratic_drag is > 0 (a positive
        coefficient is almost always a sign error, and it makes the model
        unstable).
    """
    raise NotImplementedError("load_params")


def mass_matrix(p: VehicleParams) -> np.ndarray:
    """System inertia matrix M = M_RB + M_A, about the body origin, body frame.

    Rigid body, with the centre of gravity at r_g (body frame) and
    I_O = p.inertia about the origin:

        M_RB = [[ m * I3,          -m * skew(r_g) ],
                [ m * skew(r_g),    I_O           ]]

    Added mass (diagonal; off-diagonal added-mass terms are not modelled):

        M_A = -diag(p.added_mass)

    Parameters
    ----------
    p : VehicleParams
        Vehicle parameters (not the position).

    Returns
    -------
    np.ndarray, shape (6, 6)
        M, units kg in the top-left block, kg m in the off-diagonal blocks,
        kg m^2 in the bottom-right block. Symmetric whenever p.inertia is
        (M == M.T exactly, since skew(r_g).T == -skew(r_g)) and positive
        definite for physically consistent parameters. It does not depend
        on the state, so callers may compute it once and cache it.
    """
    raise NotImplementedError("mass_matrix")


def coriolis(p: VehicleParams, nu) -> np.ndarray:
    """Coriolis and centripetal matrix C(nu) = C_RB(nu) + C_A(nu), body frame.

    Built from the full mass matrix M = mass_matrix(p) with the
    skew-symmetric parametrization (Fossen's rigid-body kinetics chapter;
    Sagatun and Fossen, 1991). Split M into 3x3 blocks,
    M11 = M[0:3, 0:3], M12 = M[0:3, 3:6], M21 = M[3:6, 0:3],
    M22 = M[3:6, 3:6], and nu = [v; w]. Then

        a = M11 v + M12 w        (3,), linear momentum-like term
        b = M21 v + M22 w        (3,), angular momentum-like term

        C(nu) = [[ 0_3,        -skew(a) ],
                 [ -skew(a),   -skew(b) ]]

    so that C(nu) nu = [w x a; v x a + w x b]. Because the formula is linear
    in M, applying it to M = M_RB + M_A gives C_RB + C_A at once. The
    parametrization assumes M is symmetric (M21 == M12.T), which holds when
    p.inertia is symmetric.

    Parameters
    ----------
    p : VehicleParams
        Vehicle parameters (not the position).
    nu : array-like, shape (6,)
        Body velocity [v; w], m/s and rad/s.

    Returns
    -------
    np.ndarray, shape (6, 6)
        C(nu). It is skew-symmetric, C == -C.T, so nu^T C(nu) nu == 0: the
        Coriolis terms do no work and neither add nor remove kinetic energy.
        nu = 0 gives the zero matrix.
    """
    raise NotImplementedError("coriolis")


def damping(p: VehicleParams, nu) -> np.ndarray:
    """Hydrodynamic damping matrix D(nu), diagonal, body frame.

        D(nu) = -diag(p.linear_drag + p.quadratic_drag * |nu|)

    where |nu| is the elementwise absolute value. The damping term on the
    left side of the equation of motion is D(nu) nu. For surge, for example,
    that term is -(X_u + X_u|u| |u|) u, so the drag force itself,
    (X_u + X_u|u| |u|) u, opposes u.

    Parameters
    ----------
    p : VehicleParams
        Vehicle parameters (not the position).
    nu : array-like, shape (6,)
        Body velocity [v; w], m/s and rad/s. Velocity relative to the water:
        no current is modelled.

    Returns
    -------
    np.ndarray, shape (6, 6)
        D(nu), diagonal with every entry >= 0 (positive semi-definite) when
        the coefficients follow the Fossen sign (<= 0). nu = 0 gives
        -diag(p.linear_drag). Units N s/m (linear rows) and N m s/rad
        (angular rows).
    """
    raise NotImplementedError("damping")


def restoring(p: VehicleParams, q) -> np.ndarray:
    """Gravity and buoyancy vector g(q), body frame, as it appears on the LEFT side.

    With e3 = [0, 0, 1] (world up, z UP) and e3_b = R(q)^T e3 (world up
    expressed in the body frame, a unit 3-vector):

        g(q) = [ (W - B) e3_b ;
                 W r_g x e3_b - B r_b x e3_b ]

    W = p.weight, B = p.buoyancy. Derivation: gravity acts at r_g with body
    force -W e3_b, buoyancy at r_b with body force +B e3_b; their sum
    [(B - W) e3_b; -W r_g x e3_b + B r_b x e3_b] is the external force and
    moment about the body origin, and moving it to the left side flips the
    sign. This is the z-up form: Fossen's NED expressions differ in the
    signs of the y and z components, because those axes point the other way.

    e3_b can be read directly from q = [x, y, z, w] (the third row of R(q)):

        e3_b = [2 (x z - y w),  2 (y z + x w),  1 - 2 (x^2 + y^2)]

    or computed as quat_to_R(q).T @ e3.

    Parameters
    ----------
    p : VehicleParams
        Vehicle parameters (not the position).
    q : array-like, shape (4,)
        Attitude, body to world, scalar-last [x, y, z, w]. Must have unit
        norm (within about 1e-9); it is not renormalized here.

    Returns
    -------
    np.ndarray, shape (6,)
        g(q): force in N (first three), moment about the body origin in N m
        (last three), body frame.

        Checks:
          * Level (q = [0, 0, 0, 1]): e3_b = [0, 0, 1], force part
            [0, 0, W - B]. With B > W this is negative; with r_g and r_b on
            the body z axis the moment part is zero, so at rest with zero
            thrust nu_dot = -M^-1 g has a positive z component: the vehicle
            accelerates up.
          * W == B, r_g directly below r_b, and the nose tilted down
            (positive rotation about body +y): the moment part has a
            positive y component, so -g turns the nose back up (righting
            moment).
          * W == B and r_g == r_b gives zeros for any q.
    """
    raise NotImplementedError("restoring")


def body_accel(p: VehicleParams, q, nu, tau) -> np.ndarray:
    """Body-frame acceleration from the equation of motion.

        nu_dot = M^-1 (tau - C(nu) nu - D(nu) nu - g(q))

    with M = mass_matrix(p), C = coriolis(p, nu), D = damping(p, nu),
    g = restoring(p, q). Solve the linear system with np.linalg.solve(M, rhs)
    rather than forming M^-1 explicitly.

    nu_dot is the time derivative of the body-frame components of nu. It is
    not the world-frame acceleration and not what an accelerometer reads
    (that also includes w x v and gravity).

    Parameters
    ----------
    p : VehicleParams
        Vehicle parameters (not the position).
    q : array-like, shape (4,)
        Attitude, body to world, scalar-last [x, y, z, w], unit norm. Only
        the restoring term uses it.
    nu : array-like, shape (6,)
        Body velocity [v; w], m/s and rad/s.
    tau : array-like, shape (6,)
        Applied force (N) and torque about the body origin (N m), body
        frame. Convert thruster commands to SI before calling.

    Returns
    -------
    np.ndarray, shape (6,)
        nu_dot = [v_dot; w_dot], m/s^2 and rad/s^2, body frame.

        At rest (nu = 0), level, tau = 0, with r_g and r_b both on the body
        z axis: nu_dot = M^-1 [0, 0, B - W, 0, 0, 0]. When r_g is not zero,
        M couples the linear and angular rows, so a pure force can also
        produce angular acceleration.

    Raises
    ------
    numpy.linalg.LinAlgError
        If M is singular (non-physical parameters).
    """
    raise NotImplementedError("body_accel")
