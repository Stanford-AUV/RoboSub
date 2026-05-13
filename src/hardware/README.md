# Package: `hardware`

**Build type:** `ament_python`  
**Role:** Vehicle I/O — IMU bridging, DVL serial driver, thrust → PWM, Teensy/Arduino serial, optional plotting and localization test utilities. Uses `hardware/sensors.yaml` (via `GenericSensor`) for which axes/covariances are active.

## Executables (`ros2 run hardware <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `thrusters` | `nodes/thrusters.py` | Sub: `ThrustsStamped` `/thrusts`. Sub: `SensorsStamped` `sensors`. Pub: `PWMsStamped` `pwms`. Converts thrusts to PWM using `hardware.utils.thrust_generator`. |
| `imu` | `nodes/imu.py` | Sub: `sensor_msgs/Imu` `/imu/data`. Pubs (if enabled in YAML): `/rotation`, `/angular`, `/accel` with gyro bias fix and covariances. |
| `dvl` | `nodes/dvl.py` | Serial DVL via `dvl_utils`. Pubs: `/velocity`, `/position` (`TwistWithCovarianceStamped` / `PoseWithCovarianceStamped`) in **base_link** when configured. |
| `sensors` | `nodes/sensors.py` | Sub: `msgs/DVLData` on topic **`dvl`**. Pubs: `/dvl/twist_sync`, `/depth/pose_sync`. **Note:** top-of-file comment says not used in current EKF wiring; also current `dvl` node does **not** publish `DVLData` — integration gap if you rely on this node. |
| `arduino` | `nodes/arduino.py` | Sub: `pwms`, `light`. Pub: `/arduino/sensors`. Serial `/dev/ttyACM0` @ 9600. |
| `imu_plot`, `imu_plot_orientation`, `dvl_plot`, `sensors_plot` | respective `nodes/*_plot.py` | Matplotlib / debug visualization. |
| `localization_test`, `localization_plot` | `nodes/localization_test.py`, `localization_plot.py` | Localization experiments / plots. |

**Not registered as ROS executables** (present as scripts but not in `setup.py` entry_points): `nodes/depth_sensor.py` — expects `from generic_sensor import GenericSensor` (import path likely wrong for `ros2 run`); intended to read depth from `/arduino/sensors` and publish `/position`.

## Utilities

- `hardware/utils/thrusters.py` — thrust → PWM mapping used by `thrusters` node.
- `hardware/utils/dvl_utils/` — serial protocol, packets, DVL connection helpers.
- `hardware/thrusters.yaml` — thruster frame positions/orientations (also read by `control/thrust_generator`).

## Launch files in tree (not installed)

`setup.py` does **not** install `launch/` into `share/hardware`. These files exist for reference or manual `ros2 launch` with path hacks:

- `hardware/launch/test_imu.py`
- `launch/track_localization.py`

To use them from install, add a `data_files` glob for `launch/*.py` in `setup.py`.

## Parameters

Nodes expect parameters from `main/launch/params/global.yaml` when launched from `main` (e.g. `timer_period`, `history_depth`, `thruster_count`).
