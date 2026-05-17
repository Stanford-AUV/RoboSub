# Preflight: hardware sensor check

Use this before pool runs to confirm IMU and DVL data are flowing through the stack you actually launch in the field (`main` + `hardware`).

## Prerequisites

- Workspace built and sourced: `source /path/to/ros2_ws/install/setup.bash`
- **`xsens_mti_ros2_driver`** available in the same overlay (required by `hardware.py`; launch fails otherwise).
- IMU on USB (Xsens), DVL serial path as configured in your params, Arduino/Teensy on **`/dev/ttyACM0`** (or adjust your setup if different).
- A display for matplotlib (local screen or SSH with X11 forwarding). `sensors_plot` uses an interactive window.

## Run (one command)

```bash
ros2 launch main hardware.py plot:=true
```

This starts Xsens, **`hardware/imu`**, **`hardware/dvl`**, thrust pipeline, thrusters, Arduino, and **`hardware/sensors_plot`** (only when `plot:=true`).

## What to verify (plots)

`sensors_plot` subscribes **`/accel`**, **`/angular`**, **`/velocity`** and shows three live traces:

| Plot | Source | Quick check |
|------|--------|-------------|
| IMU acceleration | `/accel` (`sensor_msgs/Imu`) | Move/tilt the vehicle: traces should respond; at rest, Z should reflect gravity in the sensor frame (order of ~9.8 m/s² depending on mounting). |
| IMU angular rate | `/angular` | Rotate the hull slowly: X/Y/Z rad/s should move and return near zero when still. |
| DVL velocity | `/velocity` | On the bench, velocities are often near zero. In water, with a valid bottom lock, horizontal components should track motion; no lock often means flat lines. |

If a subplot stays empty, that topic is not publishing (trace IMU bridge vs DVL serial vs permissions).

## Optional: no GUI

Skip plots and use the CLI:

```bash
ros2 launch main hardware.py
```

Then in another terminal (after `source install/setup.bash`):

```bash
ros2 topic hz /accel /angular /velocity
```

You want stable non-zero rates for IMU topics when the robot is powered; DVL rate depends on firmware and environment.

## Optional: Arduino / depth path

If you use depth from the Arduino path:

```bash
ros2 topic echo /arduino/sensors --once
```

Confirm you get a message and sensible fields for your wiring.

## Safety

- Prefer **dry** checks first: props clear, no unintended PWM, thrust commands at zero until you intentionally teleop.
- `hardware.py` includes **`thrust_generator`** and **`thrusters`**; treat this like a powered vehicle, not a passive logger.

## Common failures

| Symptom | Likely cause |
|---------|----------------|
| Launch error about `xsens_mti_ros2_driver` | Package not in workspace or overlay; install or merge the driver package. |
| IMU plots flat, no `/imu/data` | USB, permissions, or Xsens node not running. |
| DVL flat, no `/velocity` | Serial port, cable, or DVL not powered / not locked (acoustic). |
| Plot window does not appear | No `DISPLAY`; run on robot monitor or enable X11 forwarding. |
| Permission denied on serial | Add user to `dialout` (or equivalent) and re-login. |

For full localization (EKF + `sensors`), use `ros2 launch main localization.py` in a separate session when you are ready to validate **`/odometry/filtered`**, not for this minimal sensor smoke test.
