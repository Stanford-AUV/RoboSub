# `xsens_mti_ros2_driver` (vendored)

**Build type:** `ament_cmake` (C++) · **Upstream:** Xsens / Movella MTi ROS 2 driver
**Depends on:** `nmea_msgs`, `mavros_msgs`, `geographic_msgs`, `sensor_msgs`, `rclcpp`

Third-party driver for the **Xsens MTi-200 VRU**, the vehicle's primary IMU, vendored into
this workspace. Upstream documentation is in [`README.txt`](README.txt);
[`LICENSE.txt`](LICENSE.txt) is the vendor's. **Treat it as read-only** — local changes are
hard to carry across vendor updates.

> This directory is matched by `.gitignore` (`src/xsens_mti_ros2_driver/*`) but its files
> are tracked, so a fresh clone gets the driver without extra steps.

---

## Entry points

| Executable | Role |
|---|---|
| `xsens_mti_node` | the driver — opens the MTi over serial, publishes its data streams |

```bash
ros2 run xsens_mti_ros2_driver xsens_mti_node --ros-args \
  --params-file "$(ros2 pkg prefix xsens_mti_ros2_driver)/share/xsens_mti_ros2_driver/param/xsens_mti_node.yaml"
```

In practice it is launched by `ros2 launch main imu.py` ([`../main`](../main/README.md)),
which passes that params file and is started ~20 s before the rest of the stack so the
onboard filter converges.

Its own launch files: [`launch/xsens_mti_node.launch.py`](launch/xsens_mti_node.launch.py)
and [`launch/display.launch.py`](launch/display.launch.py) (adds RViz with
[`rviz/`](rviz) + [`urdf/`](urdf)).

---

## Published topics

Every stream is gated by a `pub_*` flag in
[`param/xsens_mti_node.yaml`](param/xsens_mti_node.yaml). Enabled for our MTi-200:

| Topic | Type | Used by us |
|---|---|---|
| `/filter/quaternion` | `geometry_msgs/QuaternionStamped` | **yes** — the only topic the stack consumes (`hardware/imu`) |
| `/imu/data` | `sensor_msgs/Imu` | the `gui` bridge subscribes to it |
| `/filter/euler`, `/filter/free_acceleration`, `/filter/twist` | — | diagnostics |
| `/imu/acceleration`, `/imu/angular_velocity`, `/imu/mag` | `sensor_msgs/*` | diagnostics |
| `/imu/time_ref`, `imu/utctime` | `sensor_msgs/TimeReference` | diagnostics |
| `/temperature`, `/status` | `sensor_msgs/Temperature`, `XsStatusWord` | diagnostics |
| `/tf` | transform | `pub_transform: true` |

GNSS/INS-only streams (`/gnss`, `/gnss_pose`, `/filter/positionlla`, `/filter/velocity`,
`/nmea`) are enabled in the YAML but produce nothing on a VRU unit. `/pressure` and the
high-rate streams are off.

Topic names are **absolute** in the driver source (e.g.
`messagepublishers/orientationpublisher.h`), so they are not affected by node namespaces —
which is why `hardware/imu` subscribes to `/filter/quaternion` and not `/xsens/...`.

### Custom message

[`msg/XsStatusWord.msg`](msg/XsStatusWord.msg) — decoded MTi status bits.

---

## Hardware setup

The MTi needs an **out-of-tree kernel module** before the serial device appears. Install it
from the repo root (and re-run after any kernel update):

```bash
bash drivers/xsens_mt/install.sh
```

Mount alignment is burned into the device itself (`RotSensor`), not corrected in software —
see [`hardware/hardware/sensors.yaml`](../hardware/hardware/sensors.yaml) and
[ORIN_SETUP.md](../../ORIN_SETUP.md).
