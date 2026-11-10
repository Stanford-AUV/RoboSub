# `nmea_msgs` (vendored)

**Build type:** `ament_cmake` (`rosidl` interface package) · **Upstream:**
[ros-drivers/nmea_msgs](https://github.com/ros-drivers/nmea_msgs) (`ros2` branch)

Standard ROS messages for NMEA-format GNSS data. **No nodes, no entry points** — it exists
only so the interfaces are available at build time.

## Why it is vendored

`ros-humble-nmea-msgs` has **no Python 3.10 conda build**, and the workspace runs on
Humble + py3.10 (the only combination that pairs with the prebuilt Jetson GPU-torch
wheels). It is a compile-time dependency of
[`xsens_mti_ros2_driver`](../xsens_mti_ros2_driver/README.md) and is declared by
[`msgs`](../msgs/README.md), so it is built from source inside this workspace. See
[ORIN_SETUP.md](../../ORIN_SETUP.md).

## Contents

`Sentence`, `Gpgga`, `Gpgsa`, `Gpgst`, `Gpgsv`, `GpgsvSatellite`, `Gphdt`, `Gprmc`,
`Gpvtg`, `Gpzda` — see [`msg/`](msg) and upstream [`CHANGELOG.rst`](CHANGELOG.rst).

Nothing in our code uses these types directly; the Xsens driver's `/nmea` publisher does,
and only on GNSS/INS hardware we do not have. **Do not edit** — keep it identical to
upstream.
