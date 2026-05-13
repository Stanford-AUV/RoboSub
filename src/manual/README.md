# Package: `manual`

**Build type:** `ament_python`  
**Role:** Human-in-the-loop commands. **Default onboarding / vehicle teleop:** **`keyboard`** in an interactive terminal. **`joystick`** is optional (NATS + host script).

## Executables (`ros2 run manual <name>`)

| Executable | Module | Summary |
|------------|--------|---------|
| `keyboard` | `nodes/keyboard.py` | **Interactive only:** reads keys from `stdin` (`termios` on Linux). Publishes `geometry_msgs/WrenchStamped` on topic **`wrench`** → **`/wrench`** in default namespace. Default scales: `force=0.04`, `torque=0.02`; keys **w/a/s/d**, **r/f**, **q/e**, **t/g/y/h/u/j** per on-screen help. **Ctrl+C** publishes zero wrench and exits. |
| `joystick` | `nodes/joystick.py` | **NATS** client to `nats://localhost:4222`, subject **`joystick`**. Publishes **`wrench`**, **`light`**. Subscribes **`depth`** (`msgs/Float32Stamped`) for depth-hold PD. |

## Recommended flow (keyboard + thrust)

1. Start thrust allocation (and hardware as needed), e.g. `ros2 launch main manual.py` or `ros2 launch main hardware.py` (already includes `thrust_generator` — do not run **`manual.py` and `hardware.py` together** without fixing duplicate `thrust_generator`).

2. In a **second interactive terminal** on the machine where keys are pressed:

   ```bash
   source install/setup.bash
   ros2 run manual keyboard
   ```

`control/thrust_generator` subscribes to **`/wrench`**; keyboard publishes **`wrench`** (resolves to `/wrench` under namespace `/`).

## Optional: Joystick (NATS)

- `manual/utils/joystick.py` — `JoystickState` JSON encoding.
- `manual/joystick_local.py` — host pygame loop; repo **`joystick_local.sh`** runs it (expects `.local_venv`).

```bash
nats-server
ros2 launch main manual.py   # thrust only; then:
ros2 run manual joystick
# host: ./joystick_local.sh
```

## Docs

See `manual/manual.md` for narrative notes.
