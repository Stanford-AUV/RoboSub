# `localization`

**Not a colcon package.** There is no `package.xml` and no `setup.py` here, so colcon
never builds this directory and `ros2 run localization ...` does not exist. It is the
**configuration home for state estimation**: the EKF itself is the external
`robot_localization` package, installed into the conda env as
`ros-humble-robot-localization`.

State estimation is where [`hardware`](../hardware/README.md)'s sensor topics and
[`perception`](../perception/README.md)'s heading correction become the single
`/odometry/filtered` that [`control`](../control/README.md) and
[`planning`](../planning/README.md) both read.

---

## Entry point

The EKF is launched from [`main`](../main/README.md), not from here:

```bash
ros2 launch main localization.py
```

which starts `hardware/sensors`, `robot_localization/ekf_node` (named
`ekf_filter_node`) with [`localization/nodes/ekf.yaml`](localization/nodes/ekf.yaml), and
the two static TFs.

[`localization/nodes/robot_localization.py`](localization/nodes/robot_localization.py) is
an alternative standalone launch description that resolves `ekf.yaml` through
`get_package_share_directory('localization')`. **That call fails** — nothing installs a
`localization` share dir — so use `main/launch/localization.py`, which resolves the YAML
by relative path instead.

---

## `ekf_filter_node` interface

Configured in [`localization/nodes/ekf.yaml`](localization/nodes/ekf.yaml): 50 Hz,
`world_frame`/`odom_frame` = `odom`, `base_link_frame` = `base_link`, `publish_tf: true`,
`publish_acceleration: true`, `sensor_timeout: 0.2`.

### Inputs

| Slot | Topic | Type | Fused axes | Source |
|---|---|---|---|---|
| `pose0` | `/position` | `PoseWithCovarianceStamped` | **z position only** | `hardware/depth` (and `hardware/dvl` altitude) |
| `pose1` | `/heading_correction` | `PoseWithCovarianceStamped` | **yaw only**, absolute (`differential: false`) | `perception/heading_corrector` |
| `twist0` | `/velocity` | `TwistWithCovarianceStamped` | **body-frame vx, vy, vz** | `hardware/dvl` |
| `imu0` | `/imu/orientation` | `sensor_msgs/Imu` | **roll, pitch, yaw**, absolute | `hardware/imu` |

There is deliberately **no `twist1`**: `hardware/imu` forwards only the Xsens on-board
filter quaternion, marking angular velocity and linear acceleration unused
(`covariance[0] = -1`). `robot_localization` stops scanning `twistN` after `twist0`, so
adding a `twist2` without a `twist1` would be silently ignored.

### Outputs

| Topic | Type |
|---|---|
| `/odometry/filtered` | `nav_msgs/Odometry` |
| `/accel/filtered` | `AccelWithCovarianceStamped` (because `publish_acceleration: true`) |
| `/tf` | `odom → base_link` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |

`/set_pose` resets the filter — see `hardware/ekf_watchdog`, which uses it to recover from
divergence.

### Static transforms

Published by `main/launch/localization.py` (`tf2_ros/static_transform_publisher`), not by
this directory:

| TF | Translation (m) | Rotation |
|---|---|---|
| `base_link → imu_frame` | `-0.1525, -0.02, 0.1375` | identity — the mount rotation is burned into the MTi |
| `base_link → dvl_frame` | `-0.105, 0.0, -0.0625` | identity |

---

## Tuning notes

`process_noise_covariance` x/y (indices 0, 1) and vx/vy (6, 7) are lowered from the
`robot_localization` defaults to damp position oscillation caused by integrating noisy DVL
velocity. Raise them back toward the defaults if the estimate lags real motion.

Measurement covariances come from the sensor side, not from here — see
[`hardware/hardware/sensors.yaml`](../hardware/hardware/sensors.yaml).

A relevant failure mode and its mitigation are documented in
[`config/fastdds_async.xml`](../../config/fastdds_async.xml): the single-threaded EKF froze
for up to 36 s when a tethered subscriber vanished mid-run. The async Fast DDS profile that
fixes it is exported by `launch_sub.sh` and `run_experiment.sh`.

## Other files

`localization/nodes/object_localization.py` is an empty stub (a `Node` subclass whose
`__init__` is `pass`). Object localization actually lives in
[`perception/object_world_localizer`](../perception/README.md).
