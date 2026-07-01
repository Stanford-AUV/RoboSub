# Orin Setup — conda `robosub` environment (Docker-free)

How to bring up the Stanford RoboSub software stack on a fresh **Jetson AGX Orin**
in a single **conda** environment named `robosub` — **ROS 2 Humble** (via RoboStack)
+ **GPU PyTorch**, **no Docker**. This replaces the old `.devcontainer/` workflow.

It doubles as the **"kinks" log**: every non-obvious gotcha we hit is in
[§ Kinks & gotchas](#kinks--gotchas) so a new Orin doesn't re-discover them.

> **Why conda instead of Docker?** The dev container is a *single shared instance*
> binding host-exclusive resources (`--net=host`, ports 2222/4222, specific
> `/dev/tty*`, `--privileged`), so only one person can use it at a time. A conda env
> is just a prefix on disk — every SSH user can `conda activate robosub` at once.
> Only the physical hardware (one process per serial port) and the DDS graph stay shared.

---

## Platform baseline (verified on)

| | |
|---|---|
| Board | Jetson **AGX Orin 64 GB** (8 cores, 61 GB RAM) |
| JetPack / L4T | **6.2** / **R36.5.0** (kernel `5.15.185-tegra`) |
| OS | Ubuntu **22.04**, **glibc 2.35** |
| GPU arch | Orin = CUDA **sm_87** |
| CUDA / cuDNN | **12.6.68** / **9.3.0** |

---

## TL;DR (fresh Orin, same JetPack 6.2)

```bash
# 1. Miniforge (conda)
curl -fL -o /tmp/miniforge.sh \
  "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
bash /tmp/miniforge.sh -b -p $HOME/miniforge3 && source ~/miniforge3/etc/profile.d/conda.sh

# 2. ROS 2 Humble + deps, all in conda (RoboStack). Python 3.10 is REQUIRED (see Kinks).
conda create -n robosub python=3.10 -c robostack-humble -c robostack-staging -c conda-forge \
  ros-humble-desktop ros-humble-robot-localization ros-humble-tf-transformations \
  ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-geographic-msgs \
  ros-humble-unique-identifier-msgs colcon-common-extensions
conda activate robosub

# 3. Env activation hook (CUDA libs + conda libs + no user-site + isolated DDS domain)
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/robosub_env.sh <<'EOF'
export LD_LIBRARY_PATH=$CONDA_PREFIX/lib:/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}
export ROS_DOMAIN_ID=17
export PYTHONNOUSERSITE=1
EOF
conda deactivate && conda activate robosub   # re-activate to apply the hook

# 4. GPU PyTorch — prebuilt Jetson wheels (cp310 / CUDA 12.6 / glibc 2.35). Use `python -m pip`!
python -m pip install \
  https://pypi.jetson-ai-lab.io/jp6/cu126/+f/62a/1beee9f2f1470/torch-2.8.0-cp310-cp310-linux_aarch64.whl \
  https://pypi.jetson-ai-lab.io/jp6/cu126/+f/907/c4c1933789645/torchvision-0.23.0-cp310-cp310-linux_aarch64.whl
python -m pip install -r requirements.txt        # scipy, casadi, pyserial, depthai, ultralytics-thop, ...

# 5. Two message packages have no py3.10 conda build → source-build them:
git clone --depth 1 -b ros2 https://github.com/ros-drivers/nmea_msgs.git src/deps/nmea_msgs
git clone --depth 1 -b ros2 https://github.com/mavlink/mavros.git /tmp/mavros && cp -r /tmp/mavros/mavros_msgs src/deps/mavros_msgs

# 6. Hardware: IMU needs an out-of-tree kernel module (re-run after kernel updates)
bash drivers/xsens_mt/install.sh

# 7. Build + run
colcon build --packages-skip simulation --symlink-install
source install/setup.bash
conda activate robosub && ./launch_sub.sh
```

**Verify GPU:**
```bash
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# expect: 2.8.0  12.6  True  Orin
```

---

## Kinks & gotchas

### The distro/Python/glibc chain — why **Humble**, py3.10, prebuilt torch
- **JetPack 6.2 = Ubuntu 22.04 = glibc 2.35.** ROS 2 **Jazzy** is built for Ubuntu
  24.04 — that mismatch is *the entire reason the Docker container existed* (it
  supplied a 24.04 userspace). **RoboStack** ships ROS as conda packages decoupled
  from the host OS, removing that need.
- **RoboStack Jazzy is Python 3.11/3.12; RoboStack Humble has Python 3.10 builds**
  (older, packaged as `.tar.bz2` in `robostack-staging`). Only **py3.10** pairs with
  the prebuilt Jetson GPU-torch wheels, which are **cp310** for the JetPack-6 /
  glibc-2.35 line. So: **Humble + py3.10 + the prebuilt `cu126/cp310` torch** is the
  combo that needs *no* source build. (Jazzy would force building torch from source.)
- Verified working: `torch 2.8.0`, `cuda 12.6`, `is_available() True`, device `Orin`.

### `pip` shadowing — **use `python -m pip` + `PYTHONNOUSERSITE=1`**
A stray `~/.local/bin/pip` shadows the env's pip, so plain `pip install` puts
packages in `~/.local` (and skips ones it already sees there) — they then silently
fail to import once the env ignores user-site. **Always `python -m pip install`**, and
the activation hook sets `PYTHONNOUSERSITE=1` so nodes use the env's packages.

### `LD_LIBRARY_PATH` must include `$CONDA_PREFIX/lib`
colcon-built **C++** nodes (e.g. `xsens_mti_node`) link the RoboStack ROS libs in
`$CONDA_PREFIX/lib`, which conda does **not** put on `LD_LIBRARY_PATH` by default →
they die with `error while loading shared libraries: librcutils.so`. The activation
hook adds it (plus the host CUDA/tegra dirs for GPU torch).

### Message packages with no py3.10 conda build
`ros-humble-nmea-msgs`, `ros-humble-mavros-msgs`, and `ros-humble-depthai-ros` have
**0 py3.10 builds**. `nmea_msgs` + `mavros_msgs` are compile-time deps of the Xsens
driver → **source-built** into git-ignored `src/deps/` (their own deps —
geographic_msgs, unique_identifier_msgs — come from conda). `depthai-ros` is only
used by perception; use the **`depthai` pip SDK** instead.

### Workspace fixes made for the Humble/host move
- `msgs/package.xml` didn't declare `nmea_msgs`/`mavros_msgs`/`nav_msgs` (it
  `find_package`s them) → added, so colcon builds them in the right order.
- `main/package.xml` dropped its hard-dep on **`simulation`** (Gazebo-Harmonic only,
  not on the vehicle) so the runtime workspace builds. Build with
  `--packages-skip simulation`.
- `localization.py` hardcoded the Docker path `/workspaces/RoboSub/.../ekf.yaml`
  (and `sensors_plot.py` a `/workspaces/...png`) → made host-relative.

### Env rename
If you build the workspace and *then* `conda rename` the env, the install/ shebangs
point at the old env path — **rebuild** (`rm -rf build install log && colcon build …`).
Create the env as `robosub` up front to avoid this.

### Future simplification
conda-forge merged native **Tegra** PyTorch (sm_87, CUDA 12.9, py3.10–3.14). Once an
installable build ships, GPU torch could come straight from `conda install` — re-check.

---

## Hardware & udev

One process per serial port. Stable symlinks come from udev rules
(`/etc/udev/rules.d/`, `sudo udevadm control --reload`). **No passwordless sudo — a
human runs `sudo` steps.**

| Device | Chip / bus | Node | Notes |
|---|---|---|---|
| **IMU** Xsens MTi-200 | native-USB `2639:0012` | `/dev/ttyUSB_imu` | needs the **`xsens_mt` kernel module** (below); publishes `/imu/data` @ ~400 Hz |
| **DVL** Wayfinder | FTDI FT232R (`0403:6001`) | `/dev/ttyUSB1` | node auto-detects the port; add a udev rule for a stable `/dev/ttyUSB_dvl` (none exists yet) |
| **Teensy** | native-USB CDC | `/dev/ttyACM*` | thruster/servo bridge |

### IMU — install the `xsens_mt` driver (NOT ftdi_sio)
The MTi-200 is a **native-USB device, not an FTDI chip**; JetPack 6.2 ships
`CONFIG_USB_SERIAL_XSENS_MT` unset. Build/install the out-of-tree module:
```bash
bash drivers/xsens_mt/install.sh   # + udev rule for /dev/ttyUSB_imu; re-run after kernel updates
```
See [§ IMU debug history](#imu-debug-history-the--32-saga) for why ftdi_sio is wrong.

### DVL udev (recommended)
Only the IMU has a udev rule today. For a stable DVL symlink, add (sudo):
```
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="ttyUSB_dvl", MODE="0666", GROUP="dialout"
```

---

## Smoke test (what "working" looks like)

`conda activate robosub && ./launch_sub.sh` brings up hardware + localization +
control + planning. Confirmed on the bench (Teensy off):
- `/imu/data` @ **400 Hz** (xsens driver + `hardware/imu`)
- `/odometry/filtered` @ **50 Hz** (robot_localization EKF fusing)
- GPU torch usable (`torch.cuda.is_available()` True, device Orin)
- Teensy/arduino errors are expected when the Teensy is powered off.
- **DVL** connects only if it is actually streaming — a powered-but-idle DVL reads 0
  bytes and the connect fails (a DVL state issue, independent of this env).

---

## IMU debug history (the `-32` saga)

Kept so nobody re-runs a dead end. Symptom: `ftdi_sio` attached `ttyUSB0/1` but every
FTDI control transfer returned **`-32` (EPIPE/STALL)**; baud never set; garbage reads.

- **Misdiagnosis (wrong):** a tegra-xusb/kernel USB bug needing an L4T reflash or a
  true root port.
- **Actual root cause:** the MTi-200 (`2639:0012`) is a **native-USB device, not an
  FTDI chip** (`bDeviceClass=2`). It had been force-bound to `ftdi_sio` via `new_id`;
  ftdi_sio then sent FTDI *vendor* control requests to a non-FTDI device, which
  correctly **STALLs** them → the deterministic `-32`. Standard control worked; only
  FTDI-vendor control stalled — proof the *device* rejected FTDI commands.
- **Fix (verified `/imu/data` @ ~400 Hz):** the in-kernel **`xsens_mt`** driver +
  udev symlink, in `drivers/xsens_mt/` → `bash drivers/xsens_mt/install.sh`. The DVL's
  FT232R is a *real* FTDI part and keeps using `ftdi_sio`.

---

## Dependency layers (supersedes the old `DEPENDENCY_MANAGEMENT.md`)

One conda env, not the old apt + venv + `--break-system-packages` layering:
- **ROS 2 Humble + libs** → RoboStack conda packages in `robosub`.
- **GPU torch/torchvision** → prebuilt Jetson `cu126/cp310` wheels (`python -m pip`).
- **Other Python deps** → `requirements.txt` via `python -m pip` (torch intentionally
  *not* listed there, to avoid pulling a CPU wheel).
- **Camera** → `depthai` pip SDK (`depthai-ros` isn't on RoboStack for py3.10).
- **Workspace + `src/deps/` message packages** → `colcon build`.

---

## What still lives in Docker (nothing, by design)

`.devcontainer/` and `sim_docker.sh` are retained only as reference until this setup
is fully validated in the water; they can be removed once confirmed.
