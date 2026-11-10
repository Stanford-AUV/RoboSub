# `localization`

**Build type:** `ament_python` · **Depends on:** `robot_localization`, `geometry_msgs`,
`nav_msgs`, `sensor_msgs`, `rclpy`, `tf2_ros`

State estimation. This package owns the EKF configuration that turns
[`hardware`](../hardware/README.md)'s sensor topics and
[`perception`](../perception/README.md)'s heading correction into the single
`/odometry/filtered` that [`control`](../control/README.md) and
[`planning`](../planning/README.md) both read. The filter itself is the external
`robot_localization` package (conda: `ros-humble-robot-localization`); what lives here is
its tuning, plus room for nodes built on top of it.

---

## Launch

| Command | Starts |
|---|---|
| `ros2 launch main localization.py` | the full bring-up: `hardware/sensors`, `ekf_node`, and both static TFs |
| `ros2 launch localization ekf.py` | `ekf_node` alone, for bench work |

`ekf.yaml` installs to `share/localization/`, so both resolve it through
`get_package_share_directory("localization")`.

---

## `ekf_filter_node` interface

Configured in [`localization/ekf.yaml`](localization/ekf.yaml): 50 Hz,
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

`/set_pose` resets the filter.

### Static transforms

Published by [`main/launch/localization.py`](../main/launch/localization.py):

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
