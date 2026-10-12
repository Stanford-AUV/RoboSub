# `mavros_msgs` (vendored)

**Build type:** `ament_cmake` (`rosidl` interface package) · **Upstream:**
[mavlink/mavros](https://github.com/mavlink/mavros) → `mavros_msgs`

MAVLink-related ROS interfaces: 80 messages and 46 services. **No nodes, no entry
points** — interfaces only.

## Why it is vendored

Same reason as [`nmea_msgs`](../nmea_msgs/README.md): `ros-humble-mavros-msgs` has **no
Python 3.10 conda build**, and it is a compile-time dependency of
[`xsens_mti_ros2_driver`](../xsens_mti_ros2_driver/README.md), declared by
[`msgs`](../msgs/README.md). Its own dependencies (`geographic_msgs`,
`unique_identifier_msgs`) do come from conda. See [ORIN_SETUP.md](../../ORIN_SETUP.md).

We fly no MAVLink autopilot, so none of these types appear in our nodes — this package is
pure build plumbing. **Do not edit**; keep it identical to upstream
([`CHANGELOG.rst`](CHANGELOG.rst)).

See [`msg/`](msg), [`srv/`](srv), and the C++ helpers in [`include/`](include).
