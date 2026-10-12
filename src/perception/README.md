# `perception`

**Build type:** `ament_python` · **Depends on:** `msgs`, `sensor_msgs`, `vision_msgs`, `geometry_msgs`, `nav_msgs`, `cv_bridge`, `rclpy`
**Non-ROS deps:** `depthai` (OAK), `pyrealsense2` (RealSense), `ultralytics` (YOLO), `opencv`, `torch`

Cameras in, world-frame object goals out. Two pipelines live here: the **detection
pipeline** (image → YOLO → 3D point → odom-frame goal consumed by
[`planning`](../planning/README.md)) and the **heading pipeline** (bottom line angle →
absolute yaw fused by the EKF in [`localization`](../localization/README.md)).

---

## Entry points

Registered in [`setup.py`](setup.py) — run with `ros2 run perception <name>`.

| Executable | Module | Node name |
|---|---|---|
| `oak_node` | `perception/nodes/oak.py` | `oak_node` |
| `realsense_node` | `perception/nodes/realsense.py` | `realsense_node` |
| `aligned_depth_publisher` | `perception/nodes/aligned_depth_publisher.py` | `aligned_depth_publisher` |
| `object_detection` | `perception/nodes/object_detections.py` | `<camera_key>_<object_id>_detections` |
| `object_localizer` | `perception/nodes/object_localizer.py` | `object_localizer` |
| `object_world_localizer` | `perception/nodes/object_world_localizer.py` | `object_world_localizer` |
| `heading_corrector` | `perception/nodes/heading_corrector.py` | `<camera_key>_heading_corrector` |
| `camera_viewer` | `perception/nodes/camera_viewer.py` | `camera_viewer_<camera_id>` |
| `photographer` | `perception/nodes/photographer.py` | `photographer` |

---

## Pipeline

```
oak_node / realsense_node ──▶ /camera/<key>/rgb, /camera/<key>/depth
aligned_depth_publisher   ──▶ /camera/<key>/aligned  (msgs/AlignedDepthImage)
object_localizer          ──▶ detections3d           (vision_msgs/Detection3DArray)
object_world_localizer    ──▶ /object/<id>/world_position, /object/<id>/relative
heading_corrector         ──▶ /heading_correction    (fused as pose1 in ekf.yaml)
```

---

## Camera drivers

### `oak_node` / `realsense_node`
Both derive from `GenericCameraNode` and read
[`perception/cameras.yaml`](perception/cameras.yaml) from the package share dir, opening
every camera listed there (OAK by `i_mx_id`, RealSense by `i_serial_number`).

| Direction | Topic | Type |
|---|---|---|
| Publishes | `/camera/<key>/rgb` | `sensor_msgs/Image` |
| Publishes | `/camera/<key>/depth` | `sensor_msgs/Image` |

### `aligned_depth_publisher`
Publishes RGB + depth **aligned to the RGB frame** with intrinsics in one message, so the
localizer can deproject a pixel without a separate `camera_info` subscription. OAK uses
DepthAI v3 `ImageAlign`; RealSense uses `rs.align`. Grabs frames directly from the device
(it does not subscribe to ROS topics).

| Direction | Topic | Type |
|---|---|---|
| Publishes | `/camera/<camera_key>/aligned` | `msgs/AlignedDepthImage` |

**Parameters:** `camera_type` (`oak`\|`realsense`), `camera_key` (`oak_0`), `timing_info`, `log_fps_latency`.

---

## Detection and localization

### `object_localizer`
YOLO on the aligned RGB, depth lookup at the box centre, deprojection to a camera-frame 3D
point.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/camera/<camera_key>/aligned` (or `aligned_topic`) | `msgs/AlignedDepthImage` |
| Publishes | `detections3d` | `vision_msgs/Detection3DArray` |

**Parameters:** `model_name` (`yolo-Weights/yolov8n.pt`), `object_id` (`person`),
`camera_key` (`oak_0`), `aligned_topic` (`""` = derive from `camera_key`),
`visualize_camera`, `print_positions`, `timing_info`, `log_fps_latency`,
`inference_device` (`auto`\|`cuda`\|`cpu`), `display_scale`.

> **Currently disabled** in `main/launch/perception.py`: `ultralytics` is not installed in
> the `robosub` conda env, so the node dies on import. Re-enable once detection is back on
> the sub.

### `object_world_localizer`
Transforms camera-frame detections into a filtered **odom-frame** goal. Publishes only
while detections are fresh, so a subscriber can treat "message arriving" as "currently
tracked".

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/odometry/filtered` | `nav_msgs/Odometry` |
| Subscribes | `detections3d` | `vision_msgs/Detection3DArray` |
| Publishes | `/object/<object_id>/world_position` | `geometry_msgs/PointStamped` (odom, 10 Hz) |
| Publishes | `/object/<object_id>/relative` | `geometry_msgs/PointStamped` (base_link range readout) |

**Parameters** (from [`perception/object_tracking.yaml`](perception/object_tracking.yaml)):
`object_id`, `min_score`, `cam_translation`, `cam_rpy_deg`, `filter_window`,
`filter_alpha`, `max_jump_m`, `stale_sec`.
**Measure and set `cam_translation` / `cam_rpy_deg` before a water test** — identity means
"camera at the body origin looking straight forward".

### `object_detection`
Standalone YOLO-on-a-topic node, predating the aligned pipeline.

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/camera/<camera_key>/rgb` | `sensor_msgs/Image` |
| Publishes | `<box_topic>` | `vision_msgs/Detection2DArray` |

Its `main()` hardcodes the model `underwater_images.pt`, object `bolt`, camera `oak_0` —
edit the source or instantiate the class yourself.

### `heading_corrector`
Finds the dominant line on the pool bottom and turns it into an **absolute yaw**
measurement, resolving the 180° ambiguity with the IMU's current yaw. This is the only
drift-free heading source on the vehicle (the Xsens VRU has no magnetometer).

| Direction | Topic | Type |
|---|---|---|
| Subscribes | `/camera/<camera_key>/rgb` | `sensor_msgs/Image` |
| Subscribes | `/imu/orientation` | `sensor_msgs/Imu` |
| Publishes | `/heading_correction` | `geometry_msgs/PoseWithCovarianceStamped` |

Camera key is hardcoded to `oak_0` in `main()`. Run it on photo files instead of ROS for
offline tuning: `python3 heading_corrector.py pool1.jpg` writes `pool1_debug.jpg`.

---

## Debug / capture nodes

### `camera_viewer`
`ros2 run perception camera_viewer <camera_name>` — `cv2.imshow` windows. Publishes nothing.
Subscribes `/camera/<name>/rgb`, `/camera/<name>/depth` (`Image`) and
`/yolo/<name>/box_coords` (`vision_msgs/Detection2DArray`).

### `photographer`
`ros2 run perception photographer <camera_name> [...]` — saves periodic snapshots to disk.
Publishes nothing. Subscribes `/camera/<name>/rgb` (`Image`).
**Parameters:** `output_dir`, `capture_period_s`, `image_ext`.

---

## Launch files

| File | Starts |
|---|---|
| [`launch/camera.py`](launch/camera.py) | `oak_node` + `realsense_node`; optional `camera_viewer` (`camera_viewer:=true camera_names:=oak_0`) and `photographer` (`photographer:=true photographer_cameras:=... photographer_output_dir:=... photographer_period:=...`) |
| [`launch/aligned_depth_localizer.py`](launch/aligned_depth_localizer.py) | `aligned_depth_publisher` + `object_localizer`, with the full argument set (`camera_type`, `camera_key`, `model_name`, `object_id`, `visualize_camera`, `print_positions`, `timing_info`, `log_fps_latency`, `display_scale`). Note it still hardcodes `prefix=/home/ros/env/bin/python3` from the Docker era — override or drop that when running in the conda env. |
| `main/launch/perception.py` | `object_world_localizer` + `oak_node` + `heading_corrector` (`object_localizer` commented out) |

---

## Config

| File | Read by |
|---|---|
| [`perception/cameras.yaml`](perception/cameras.yaml) | `oak_node`, `realsense_node`, `aligned_depth_publisher`, `camera_viewer` — per-camera device id, resolution, fps, depth preset, intrinsics |
| [`perception/object_tracking.yaml`](perception/object_tracking.yaml) | `object_world_localizer` |

YOLO weights live in the workspace-root `yolo-Weights/` directory.

## Library code (`perception/utils/`)

`object_world.py` (camera→base→odom transforms + smoothing), `undistort.py`
(`build_remap`, `load_params`), `bounding_box.py` (oriented 3D box), `calibrate.py`
(dump OAK factory calibration). `install_dependencies.sh` installs the camera SDKs.

## Tests

`colcon test --packages-select perception`, or `./test.sh` from the workspace root.
