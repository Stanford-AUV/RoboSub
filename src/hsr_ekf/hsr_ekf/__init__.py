"""hsr_ekf: a physics-based EKF for the sub, written as a pure-Python package.

No ROS imports live here, so every module can be unit-tested on a laptop and
imported unchanged by a future ROS node. The derivations are in the
"Derivation" section of src/hsr_ekf/README.md.

Conventions, shared by every module in this package:
  * World frame ``odom`` and body frame ``base_link``: x forward, y left, z up
    (ROS REP-103). Fossen's textbook uses NED (z down); signs differ.
  * Attitude is a unit quaternion, never Euler angles. Arrays store it
    scalar-last, q = [x, y, z, w], the same order as ROS geometry_msgs and
    SciPy's Rotation.as_quat(). Hamilton product convention.
  * Rotations map body coordinates to world coordinates: v_world = R @ v_body,
    where R = R(q).
  * Rotating by a positive angle follows the right-hand rule about the axis:
    a positive rotation about body +y puts the nose DOWN.
  * State x = [p, q, v, w] (13,): position p in the world frame, attitude q
    (body to world), linear velocity v and angular velocity w in the body
    frame. The filter's error state is 12-dimensional: the attitude error is
    a 3-vector rotation in the body frame.
  * SI units inside the model (m, s, N, rad).

The filter is modular. The error-state EKF core (ekf) works with any process
model (process: constant velocity, or the hydrodynamic model) and any set of
measurement modules (measurements/: one file per sensor, sharing one
interface), so a sensor is added or removed without touching the core.

Modules:
  rotations      the skew matrix and a rotation-matrix check
  quaternions    quaternion algebra, Exp/Log, conversions to and from R
  kinematics     position and attitude rates from body velocity; lever arm
  dynamics       vehicle parameters and M, C, D, g; body acceleration
  state          state layout and the error-state operations (+) and (-)
  integrate      Euler and RK4 steps; exact quaternion step
  jacobians      numerical Jacobians, including through the error state
  kf             generic Kalman predict and update (Joseph form), NIS
  process        process-model interface; constant-velocity and hydro models
  ekf            the error-state EKF core (sensor-agnostic)
  measurements/  one module per sensor behind a shared interface
  simulate       truth simulation, synthetic sensors, NEES
"""
