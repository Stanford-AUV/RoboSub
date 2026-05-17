# Package: `perception`

**Build type:** `ament_python`  
**Role:** Cameras (OAK, RealSense), aligned RGB–depth publishing, YOLO-based object localization, recording helpers. Camera keys and parameters come from `perception/cameras.yaml` (installed to share) and `main/launch/params/global.yaml` when launched from `main`.

## Executables (`ros2 run perception <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `oak_node` | `nodes/oak.py` | DepthAI / OAK camera pipeline. |
| `realsense_node` | `nodes/realsense.py` | Intel RealSense pipeline. |
| `aligned_depth_publisher` | `nodes/aligned_depth_publisher.py` | Publishes `msgs/AlignedDepthImage` for a configured `camera_type` / `camera_key`. |
| `object_localizer` | `nodes/object_localizer.py` | Subscribes to aligned depth stream; runs detector (Ultralytics YOLO); publishes 3D detections (`vision_msgs` / custom points per implementation). |
| `object_detection` | `nodes/object_detections.py` | Detection pipeline entry (module name `object_detections`). |
| `camera_viewer` | `nodes/camera_viewer.py` | CLI arg: camera name(s). Viewer for debugging. |
| `photographer` | `nodes/photographer.py` | Periodic disk capture; parameters `output_dir`, `capture_period_s`. |

## Launches (`share/perception/launch/`)

- **`camera.py`** — Starts **`oak_node`** and **`realsense_node`** together; optional `camera_viewer` and `photographer` via launch arguments (`camera_viewer`, `camera_names`, `photographer`, etc.). Uses `global.yaml` from `main`.
- **`aligned_depth_localizer.py`** — **`aligned_depth_publisher`** + **`object_localizer`**, with `prefix="/home/ros/env/bin/python3 -u "` for venv Python (GPU stack). Declares args: `camera_type`, `camera_key`, `model_name`, `object_id`, visualization flags.

## Architecture (as implemented)

Producers (OAK / RealSense / aligned publisher) output **`msgs/AlignedDepthImage`** on per-camera topics. **`object_localizer`** consumes that message type and runs YOLO; **torch / ultralytics** are expected system-wide (see root [DEPENDENCY_MANAGEMENT.md](../../DEPENDENCY_MANAGEMENT.md)), not pinned in `setup.py` install_requires.

## Other files

- `perception/app.py`, `stream_over_ethernet.py` — auxiliary / experimental scripts.
- `perception/utils/` — bounding boxes, calibration JSON, tests.

## Tests

`perception/test/` — copyright/lint stubs and `utils/test/` unit tests.
