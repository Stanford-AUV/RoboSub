# `vendor`

Third-party ROS 2 packages copied into the workspace. **Do not edit them** — treat
each as read-only upstream code.

| Package | Upstream | Why it is here |
|---|---|---|
| `xsens_mti_ros2_driver` | Xsens/Movella | Driver for the MTi-200 VRU. Publishes `/filter/quaternion`, which `hardware/imu` forwards to the EKF. |
| `mavros_msgs` | [mavros](http://wiki.ros.org/mavros_msgs) v2.14.0 | Build dependency of the Xsens driver. No Python 3.10 conda build exists. |
| `nmea_msgs` | [ros-drivers/nmea_msgs](https://github.com/ros-drivers/nmea_msgs) v2.1.0 | Same reason. |

`build.sh` passes `--base-paths .`, so colcon finds these here exactly as it did under
`src/`. Everything else refers to them by package name (`ros2 run`,
`get_package_share_directory`, `find_package`), never by path.

The out-of-tree Xsens **kernel** module is separate — see
[`drivers/xsens_mt`](../drivers/xsens_mt/README.md).
