# `hardware`

**Build type:** `ament_python` · **Depends on:** `msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `rclpy`

Every node that talks to a physical device. Sensors come **in** (IMU, DVL, depth,
hydrophones, Teensy telemetry) and actuator commands go **out** (thruster PWM, lights,
torpedoes). Its outputs are exactly what the EKF in
[`localization`](../localization/README.md) fuses; its input `/thrusts` comes from
[`control`](../control/README.md).

---

## Entry points

Registered in [`setup.py`](setup.py) — run with `ros2 run hardware <name>`.

| Executable | Module | Role |
|---|---|---|
| `imu` | `hardware/nodes/imu.py` | Xsens filter quaternion → EKF orientation |
| `bno085_0`, `bno085_1` | `hardware/nodes/bno085.py` (`main_0` / `main_1`) | Secondary BNO085 IMUs via the Teensy |
| `dvl` | `hardware/nodes/dvl.py` | Teledyne Wayfinder DVL driver |
| `depth` | `hardware/nodes/depth_sensor.py` | Pressure → depth pose |
| `arduino` | `hardware/nodes/arduino.py` | Teensy serial bridge (PWM out, telemetry in) |
| `thrusters` | `hardware/nodes/thrusters.py` | Thrust (N) → PWM (µs) |
| `daisy` | `hardware/nodes/daisy.py` | DaisySeed hydrophone pinger detection |
| `sensors` | `hardware/nodes/sensors.py` | Legacy DVL→EKF repackaging (unused) |
| `ekf_watchdog` | `hardware/nodes/ekf_watchdog.py` | Detects and resets EKF divergence |
| `sensors_plot` | `hardware/nodes/sensors_plot.py` | Full-stack dashboard (web + PNG) |
| `imu_plot`, `imu_plot_orientation`, `dvl_plot`, `localization_plot` | `hardware/nodes/*_plot.py` | Single-signal matplotlib debug plots |
| `localization_test` | `hardware/nodes/localization_test.py` | Synthetic sensor injection / odometry monitor |

---

## Actuation chain

```
/wrench ──(control/thrust_generator)──▶ /thrusts ──(thrusters)──▶ pwms ──(arduino)──▶ Teensy ──▶ ESCs
```

### `thrusters`
| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/thrusts` | `msgs/ThrustsStamped` |
| Subscribes | `sensors` (relative → `/sensors`) | `msgs/SensorsStamped` (battery voltage → PWM curve) |
| Publishes | `pwms` | `msgs/PWMsStamped` |

**Parameters:** `timer_period` (default `0.01`), `history_depth` (default `10`).
Thrust→PWM coefficients are per battery voltage in `hardware/utils/thrusters.py`.

> **Topic mismatch:** this node subscribes to the *relative* name `sensors` (→ `/sensors`),
> but `arduino` publishes `/arduino/sensors`, and no launch file remaps either. Nothing
> publishes `/sensors`, so `sensors_callback` never fires and the PWM curve runs on the
> hardcoded fallback `14.8 V` for the whole mission. Fix with a remap in
> `main/launch/hardware.py` or by changing the topic in the node — but re-check the thrust
> calibration either way, since every pool result so far was produced at 14.8 V.

### `arduino`
Serial bridge to the Teensy on the udev symlink **`/dev/ttyACM_teensy`** at 9600 baud
(never a bare `/dev/ttyACM<n>` — re-enumeration once put a hydrophone board on `ttyACM0`).

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `pwms` | `msgs/PWMsStamped` |
| Subscribes | `light` | `std_msgs/Int16` |
| Subscribes | `/torpedo`, `/torpedo_left`, `/torpedo_right` | `std_msgs/String` |
| Publishes | `/arduino/sensors` | `msgs/SensorsStamped` |
| Publishes | `/arduino/imu_0`, `/arduino/imu_1` | `sensor_msgs/Imu` |

**Parameters:** `history_depth` (10), `thruster_count` (8).
**Env:** `NO_THRUST=1` (or `./launch_sub.sh --no-thrust`) pins every thruster PWM at
neutral 1500 while the rest of the node runs normally.

---

## Sensing chain

All sensor nodes derive from `GenericSensor` (`hardware/nodes/generic_sensor.py`), which
reads [`hardware/sensors.yaml`](hardware/sensors.yaml) from the package share dir to get,
per sensor: which data types are active (`position`/`rotation`/`velocity`/`angular`/`accel`,
encoded `0` = off, `3` = z only, `123` = xyz), the covariance matrices, the mount offset
`sensor_pos_in_base`, and the rotation `R_sensor_to_base`.

### `imu`
Republishes the Xsens on-board filter quaternion (already vehicle-aligned — the mount
rotation is burned into the MTi) as a plain `Imu` with initial heading zeroed. Angular
velocity and linear acceleration are flagged unused (`cov[0] = -1`).

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/filter/quaternion` | `geometry_msgs/QuaternionStamped` (from `xsens_mti_ros2_driver`) |
| Publishes | `/imu/orientation` | `sensor_msgs/Imu` |

### `bno085_0` / `bno085_1`
Reads the BNO085s that the Teensy forwards; publishes under the per-IMU namespace
`bno085_<i>`, gated by the `sensors.yaml` active-axis flags.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/arduino/imu_<i>` | `sensor_msgs/Imu` |
| Publishes | `bno085_<i>/rotation` | `geometry_msgs/PoseWithCovarianceStamped` |
| Publishes | `bno085_<i>/angular` | `geometry_msgs/TwistWithCovarianceStamped` |
| Publishes | `bno085_<i>/accel` | `sensor_msgs/Imu` |

### `dvl`
Teledyne Wayfinder over serial; prefers the udev symlink `/dev/ttyUSB_dvl`, falling back
to a scan of `/dev/ttyUSB*` / `/dev/ttyACM*`. Applies the ω × r lever-arm correction and
writes speed-of-sound at startup from water temperature.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/angular` | `geometry_msgs/TwistWithCovarianceStamped` (lever-arm correction) |
| Subscribes | `/arduino/sensors` | `msgs/SensorsStamped` (water temp → speed of sound, startup only) |
| Publishes | `/position` | `geometry_msgs/PoseWithCovarianceStamped` (altitude / z only) |
| Publishes | `/velocity` | `geometry_msgs/TwistWithCovarianceStamped` |

### `depth`
| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/arduino/sensors` | `msgs/SensorsStamped` |
| Publishes | `/position` | `geometry_msgs/PoseWithCovarianceStamped` (z only) |

### `daisy` — pinger direction
Host-side port of the DaisySeed `master_ros.cpp` firmware. Boards are found by USB serial
under `/dev/serial/by-id`; both 96 kHz PCM streams are decoded, run through the exact
firmware FFT detector (`hardware/pinger/`), and recorded to
`data/audio_pingers/<session>/ch<C>.wav`.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/pinger/listening` | `std_msgs/String` (`"start"` / `"stop"` from planning's branch window) |
| Publishes | `/pinger` | `std_msgs/String` — one `"front"`/`"back"` per decided ping |
| Publishes | `/pinger/levels` | `msgs/PingerStamped` — ~20 Hz per-channel peak level + latched direction |

### `ekf_watchdog`
Resets `robot_localization` when it clearly diverges: any non-finite odometry value, or
orientation disagreement with the IMU beyond a threshold sustained past a timeout. The
reset keeps position and snaps orientation to the IMU.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/rotation` | `geometry_msgs/PoseWithCovarianceStamped` |
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Publishes | `/set_pose` | `geometry_msgs/PoseWithCovarianceStamped` |

### `sensors` (legacy, not in the flown stack)
Repackages `dvl` into sync topics; `ekf.yaml` no longer reads them.
Subscribes `dvl` (`msgs/DVLData`); publishes `/dvl/twist_sync`
(`TwistWithCovarianceStamped`) and `/depth/pose_sync` (`PoseWithCovarianceStamped`).

---

## Visualization nodes

### `sensors_plot` — the run dashboard
Headless matplotlib (`Agg`) plus a stdlib HTTP server on **port 8080**
(`hardware/utils/dashboard.py`) so the laptop renders the panels:
**http://192.168.2.2:8080**. Publishes nothing. Subscribes:

`/imu/orientation` (`Imu`) · `/velocity` (`TwistWithCovarianceStamped`) ·
`/odometry/filtered` (`Odometry`) · `/accel/filtered` (`AccelWithCovarianceStamped`) ·
`/desired/pose` (`Odometry`) · `/arduino/sensors` (`SensorsStamped`) ·
`/pinger/levels` (`PingerStamped`) · `/mission/stage` (`String`)

### Single-signal plots (all `TkAgg`, need a `DISPLAY`)

| Node | Subscribes |
|---|---|
| `imu_plot` | `imu` (`Imu`) |
| `imu_plot_orientation` | `imu`, `/imu/pose_sync` (`Imu`) |
| `dvl_plot` | `dvl` (`msgs/DVLData`) |
| `localization_plot` | `/odometry/filtered` (`Odometry`) |
| `localization_test` | `/odometry/filtered`; in `TEST_MODE` also **publishes** `/imu/data` (`Imu`), `dvl` (`DVLData`), `depth` (`Float32Stamped`) |

---

## Launch files

The vehicle launch files live in [`main`](../main/README.md), not here:

- `main/launch/imu.py` — `xsens_mti_node` + `imu` (brought up first so the Xsens filter settles)
- `main/launch/hardware.py` — `dvl`, `control/thrust_generator`, `thrusters`, `arduino`, `daisy`, `depth`, and `sensors_plot` (argument `plot:=true` by default)

`hardware/hardware/launch/test_imu.py` is a bench IMU-only launch (not installed).

---

## Config

| File | Read by |
|---|---|
| [`hardware/sensors.yaml`](hardware/sensors.yaml) | every `GenericSensor` subclass — covariances, mounts, active axes |
| [`hardware/thrusters.yaml`](hardware/thrusters.yaml) | `control/thrust_generator` — thruster positions and thrust directions in body frame (x fwd, y port, z up) |

Both install to `share/hardware/`, which is where the nodes read them from — edit the
source copy, then **rebuild**, or the old values stay in effect.

## Vendored libraries

`hardware/utils/dvl_utils/` is Teledyne RD Instruments' Wayfinder SDK (unmodified).
`hardware/pinger/` is a line-by-line port of the DaisySeed firmware's detection path —
do not "clean it up": the point is bit-level parity with the firmware.

## Tests

`colcon test --packages-select hardware`, or `./test.sh` from the workspace root.
