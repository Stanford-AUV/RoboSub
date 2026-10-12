# `manual`

**Build type:** `ament_python` · **Depends on:** `msgs`, `geometry_msgs`, `std_msgs`, `rclpy`
**Non-ROS deps:** `nats-py`, `dataclasses-json` (vehicle side); `pynput`, `pygame` (operator side)

Operator override. A keyboard or gamepad on the **laptop** publishes its state to a
**NATS** broker; a ROS node on the **sub** turns that into a `/wrench` — the same topic
[`control`](../control/README.md)'s `pid_control` writes, so teleop and autonomy must never
run at the same time.

NATS (rather than DDS) keeps the operator machine out of the ROS graph: no shared
`ROS_DOMAIN_ID`, no multicast discovery over the tether.

---

## Entry points

Registered in [`setup.py`](setup.py) — run with `ros2 run manual <name>`.

| Executable | Module | Node name |
|---|---|---|
| `keyboard` | `manual/nodes/keyboard.py` | `keyboard_node` |
| `joystick` | `manual/nodes/joystick.py` | `joystick_node` |
| `prequal` | `manual/nodes/prequal.py` | `keyboard_node` |

Operator-side scripts are **not** ROS nodes — run them from the repo root on the laptop:
[`keyboard_local.sh`](../../keyboard_local.sh) → `manual/keyboard_local.py`, and
[`joystick_local.sh`](../../joystick_local.sh) → `manual/joystick_local.py`. Both expect a
`.local_venv` built from [`local_requirements.txt`](../../local_requirements.txt).

---

## Nodes

### `keyboard`
Subscribes the **NATS** subject `keyboard` at `nats://localhost:4222`, deserializes a
`KeyboardState` (`manual/utils/keyboard.py`), and republishes it as ROS messages.

| Direction | Topic / subject | Type |
|---|---|---|
| Subscribes | NATS `keyboard` | `KeyboardState` JSON |
| Publishes | `wrench` | `geometry_msgs/WrenchStamped` |
| Publishes | `light` | `std_msgs/Int16` |
| Publishes | `torpedo` | `std_msgs/Int16` |
| Publishes | `dropper` | `std_msgs/Int16` |

### `joystick`
| Direction | Topic / subject | Type |
|---|---|---|
| Subscribes | NATS `joystick` | `JoystickState` JSON |
| Subscribes | `depth` | `msgs/Float32Stamped` |
| Publishes | `wrench` | `geometry_msgs/WrenchStamped` |
| Publishes | `light` | `std_msgs/Int16` |

### `prequal`
Scripted prequalification run: replays the timed key sequence in
[`manual/prequalPresses.yaml`](manual/prequalPresses.yaml) instead of reading NATS (the
path is resolved relative to the source file, so it needs `--symlink-install` or a source
checkout). Publishes `wrench` (`WrenchStamped`) and `light` (`Int16`).

---

## Key map

Defined in `manual/keyboard_local.py`:

| Keys | Action |
|---|---|
| `w` / `s` | forward / back |
| `a` / `d` | strafe left / right |
| `r` / `f` | up / down |
| `q` / `e` | yaw CCW / CW |
| `t` / `g` | overall velocity scale |
| `y` / `h` | linear scale |
| `u` / `j` | angular scale |

State is sent to NATS every 50 ms.

---

## Running teleop

```bash
# on the sub
nats-server &                    # must be reachable from the laptop too
ros2 launch main hardware.py     # starts control/thrust_generator, thrusters, arduino
ros2 run manual keyboard

# on the laptop, from the repo root
./keyboard_local.sh
```

Bench alternative: `ros2 launch main manual.py` starts `thrust_generator` + `keyboard`
together. **Do not run it alongside `hardware.py`** — both start `thrust_generator`.

Data path:

```
keyboard_local (pynput) ──NATS "keyboard"──▶ manual/keyboard ──/wrench──▶ thrust_generator ──/thrusts──▶ thrusters ──pwms──▶ arduino ──▶ ESCs
```

Stop teleop before launching autonomy (`./launch_sub.sh` / `ros2 launch main main.py`), or
two publishers will contest `/wrench`.

## Library code (`manual/utils/`)

`keyboard.py` and `joystick.py` define the `@dataclass_json` wire formats shared by the
operator scripts and the ROS nodes — change both sides together.

## Tests

`colcon test --packages-select manual`, or `./test.sh` from the workspace root.
