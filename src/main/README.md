# Package: `main`

**Build type:** `ament_cmake`  
**Role:** Installs top-level launch files and shared parameters. This package does not contain application logic; it composes other packages.

## What is installed

- `share/main/launch/*.py` — system bring-up
- `share/main/launch/params/global.yaml` — shared parameters (thruster geometry, EKF `robot_localization` config, camera snippets, timer/history defaults)

## Launch files (as coded)

| Launch file | Behavior |
|-------------|----------|
| `main.py` | Includes `hardware.py`, `localization.py`, `manual.py`. Autonomous `control` stack is commented out. |
| `hardware.py` | `xsens_mti_ros2_driver` + `hardware` nodes (`imu`, `dvl`, `thrusters`, `arduino`) + `control/thrust_generator`; optional `sensors_plot` via launch arg `plot`. Requires **xsens** package in workspace/overlay. |
| `localization.py` | `hardware/sensors` + `robot_localization/ekf_node` + static TFs `base_link` → `imu_frame`, `dvl_frame`. |
| `state.py` | Same EKF stack as `localization.py` without manual/hardware extras. |
| `manual.py` | **`control/thrust_generator` only.** Run **`ros2 run manual keyboard`** in a separate interactive terminal for teleop (keyboard uses `stdin`; it is not launched here). |
| `control.py` | `control/thrust_generator` + `control/controller` + `control/test_controller` (publishes sample `waypoint` odometry). |
| `perception.py` | Single node: `perception/object_localizer`. |
| `planning.py` | Empty `LaunchDescription` (placeholder). |
| `simulation.py` | `simulation/thrusters`, `simulation/sensors`, `simulation/path_bridge` with `global.yaml`. |

## How to run

From workspace root after `source install/setup.bash`:

```bash
ros2 launch main main.py
ros2 launch main control.py
ros2 launch main hardware.py
# …etc.
```

## Dependencies (package.xml)

`control`, `hardware`, `manual`, `perception`, `planning`, `simulation`, `gui`, plus standard message/runtime packages. External stacks (e.g. `xsens_mti_ros2_driver`, `robot_localization`) are referenced from launch files but not always listed as `exec_depend` here.
