# `msgs`

**Build type:** `ament_cmake` (`rosidl` interface package)
**Depends on:** `std_msgs`, `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `vision_msgs`, `builtin_interfaces`, `nmea_msgs`, `mavros_msgs`

Every custom interface in the workspace. This package has **no nodes and no entry
points** — it generates Python/C++ types that the other packages import. Any new
cross-package type belongs here; rebuild before using it
(`colcon build --packages-select msgs && source install/setup.bash`).

Interfaces are registered in [`CMakeLists.txt`](CMakeLists.txt) via
`rosidl_generate_interfaces`. Adding a `.msg` file without adding it there generates
nothing.

---

## Messages

### Actuation

| Message | Fields | Flows |
|---|---|---|
| [`ThrustsStamped`](msg/ThrustsStamped.msg) | `header`, `float64[] thrusts` | `control/thrust_generator` → `hardware/thrusters`, `simulation/thrusters` |
| [`PWMsStamped`](msg/PWMsStamped.msg) | `header`, `int16[] pwms` | `hardware/thrusters` → `hardware/arduino` |

### Vehicle telemetry

| Message | Fields | Flows |
|---|---|---|
| [`SensorsStamped`](msg/SensorsStamped.msg) | `header`, `pressure`, `depth`, `external_temperature`, `internal_temperature1`, `internal_temperature2`, `humidity`, `current`, `voltage` | `hardware/arduino` on `/arduino/sensors` → `depth`, `dvl`, `sensors_plot` (`thrusters` wants it too but listens on the wrong topic — see [hardware](../hardware/README.md)) |
| [`Float32Stamped`](msg/Float32Stamped.msg) | `header`, `float32 data` | generic stamped scalar (`depth`) |
| [`PingerStamped`](msg/PingerStamped.msg) | `header`, `float32[4] levels` (normalized peak per hydrophone; front = 0, 3 / back = 1, 2), `bool front` (latched) | `hardware/daisy` → `sensors_plot` |

### DVL

| Message | Fields |
|---|---|
| [`DVLData`](msg/DVLData.msg) | `header`, `uint8 type`, `DVLTarget target`, `DVLVelocity velocity`, `DVLBeam[] beams` |
| [`DVLVelocity`](msg/DVLVelocity.msg) | `uint8 reference`, `geometry_msgs/Vector3 mean`, `float32[] covariance` |
| [`DVLBeam`](msg/DVLBeam.msg) | `uint8 id`, `DVLVelocity velocity`, `float32 range_mean`, `bool locked` |
| [`DVLTarget`](msg/DVLTarget.msg) | `uint8 type`, `float32 range_mean` |

### Perception

| Message | Fields | Flows |
|---|---|---|
| [`AlignedDepthImage`](msg/AlignedDepthImage.msg) | `sensor_msgs/Image rgb`, `sensor_msgs/Image depth`, `sensor_msgs/CameraInfo camera_info`, `builtin_interfaces/Time hardware_stamp` | `aligned_depth_publisher` → `object_localizer` |
| [`Detection3DPoints`](msg/Detection3DPoints.msg) | `header`, `vision_msgs/ObjectHypothesisWithPose[] results`, `geometry_msgs/Point[] points`, `string id` | point-cloud-carrying detection |
| [`Detection3DPointsArray`](msg/Detection3DPointsArray.msg) | `header`, `Detection3DPoints[] detections` | multi-object variant |

Depth is aligned to the RGB frame (same resolution, pixel correspondence), which is why
`AlignedDepthImage` can carry one `camera_info` for both images.

### Planning / IMU (legacy)

| Message | Fields | Status |
|---|---|---|
| [`GeneratedPath`](msg/GeneratedPath.msg) | `header`, `geometry_msgs/PoseStamped[] poses`, `geometry_msgs/Twist[] twists`, `float64 duration` | only `simulation/path_bridge` still subscribes; `path_generator` streams `nav_msgs/Odometry` on `/desired/pose` instead |
| [`MTi200Data`](msg/MTi200Data.msg) | `header`, `sensor_msgs/TimeReference imu_time`, `orientation`, `rate_of_turn`, `linear_acceleration`, `free_acceleration` | from the pre-`xsens_mti_ros2_driver` IMU bridge; unused today |

---

## Services

| Service | Request | Response | Status |
|---|---|---|---|
| [`GetWaypoints`](srv/GetWaypoints.srv) | *(empty)* | `nav_msgs/Path waypoints` | Client: `control/path_tracker`. **No server node exists in this repo**, so `path_tracker` blocks at startup waiting for it. |

---

## Dependency note

`msgs` declares `nmea_msgs` and `mavros_msgs` because the Xsens driver chain needs them,
and neither has a Python-3.10 conda build. Both are vendored into this workspace as
[`../nmea_msgs`](../nmea_msgs/README.md) and [`../mavros_msgs`](../mavros_msgs/README.md),
so colcon builds them ahead of `msgs` — see [ORIN_SETUP.md](../../ORIN_SETUP.md).
