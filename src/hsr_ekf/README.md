# `hsr_ekf`: EKF / PID with hydrodynamics

**Team work in progress on branch `HSR-EKF`.** This folder belongs to the *EKF / PID with
hydrodynamics* project. It is kept separate from the rest of the stack, and nothing outside this
folder imports it yet.

The goal is a physics-based, quaternion, error-state EKF that can replace `robot_localization` as
the owner of `/odometry/filtered` once it is validated. Model-based control on top of the PID comes
later. The package is pure Python with no ROS imports, so every module is unit-tested on a laptop
and a future ROS node can import it unchanged.

This folder is not a colcon package yet (it has no `package.xml` or `setup.py`), so colcon and
`test.sh` ignore it.

## Layout

```text
src/hsr_ekf/
  hsr_ekf/
    rotations.py      skew matrix, rotation-matrix check
    quaternions.py    quaternion algebra, Exp/Log, conversions
    kinematics.py     position and attitude rates; lever arm
    dynamics.py       M, C, D, g and body acceleration (Fossen form, z up)
    state.py          13-element state, 12-element error state, (+) and (-)
    integrate.py      Euler, RK4, exact quaternion step
    jacobians.py      numerical Jacobians, including through the error state
    kf.py             generic Kalman predict/update (Joseph form), NIS
    process.py        process-model interface: constant velocity, hydro model
    ekf.py            error-state EKF core; knows no specific sensor
    measurements/     one module per sensor: depth, DVL, IMU attitude, gyro
    simulate.py       truth simulation, synthetic sensors, NEES
  config/vehicle_model.yaml   vehicle coefficients (placeholders until fitted)
  test/                       pytest suite, one file per module
```

Functions are added module by module. A module whose functions still raise
`NotImplementedError` is not written yet.

## Conventions

- Frames are `odom` (world) and `base_link` (body), with x forward, y left, z up (REP-103). This
  is not NED.
- Attitude is a unit quaternion, never Euler angles. Arrays are scalar-last, `[x, y, z, w]`,
  matching ROS messages and SciPy.
- The state is `[p, q, v, ω]` (13 elements). Position is in the world frame; linear and angular
  velocity are in the body frame.
- The error state has 12 elements. The attitude error is a body-frame rotation vector:
  `q_true = q ⊗ Exp(δθ)`.
- Units are SI. `/wrench` may be in kgf, and that conversion belongs at the ROS boundary.
- The filter is modular. To add a sensor, add one module under `measurements/`; the core does not
  change.

## Run the tests

Needs NumPy, SciPy, and pytest. No ROS is needed.

```bash
cd src/hsr_ekf && python3 -m pytest test -q
```

## Derivation

The math behind `hsr_ekf`, one section per module group.

### 1. Frames, rotations, and quaternions

_Not written yet._

### 2. Kinematics

_Not written yet._

### 3. Dynamics

_Not written yet._

### 4. Integration and Jacobians

_Not written yet._

### 5. Kalman filter

_Not written yet._

### 6. Error-state EKF and process models

_Not written yet._

### 7. Measurement models

_Not written yet._

### 8. Consistency checks

_Not written yet._
