# Orin Setup — conda `robosub` environment (Docker-free)

How to bring up the Stanford RoboSub software stack on a fresh **Jetson AGX Orin**
in a single **conda** environment named `robosub`, with a **GPU-enabled PyTorch**,
**no Docker**. This replaces the old `.devcontainer/` Docker workflow.

It doubles as the **"kinks" log**: every non-obvious gotcha we hit is recorded in
[§ Kinks & gotchas](#kinks--gotchas) so a new Orin (or a returning teammate) doesn't
re-discover them the hard way.

> **Why conda instead of Docker?** The dev container is a *single shared instance*
> that binds host-exclusive resources (`--net=host`, published ports 2222/4222,
> specific `/dev/tty*` devices, `--privileged`), so only one person can really use
> it at a time. A conda env is just a prefix on disk — every SSH user can
> `conda activate robosub` simultaneously and work independently. Only the physical
> hardware (one process per serial port) and the DDS graph stay inherently shared.

---

## Platform baseline (what this was verified on)

| | |
|---|---|
| Board | Jetson **AGX Orin 64 GB** (8 cores, 61 GB RAM, 30 GB swap) |
| JetPack / L4T | **6.2** / **R36.5.0** (kernel `5.15.185-tegra`) |
| OS | Ubuntu **22.04** (Jammy), **glibc 2.35** |
| GPU compute | Orin = CUDA arch **sm_87** |
| CUDA / cuDNN | **12.6.68** / **9.3.0** (both dev headers + libs present under `/usr/…`) |
| System Python | 3.10 (unused by us — the conda env owns Python) |

The single most important consequence of this baseline is the **glibc/Python wall**
described in [§ Kinks](#the-gpu-torch--glibc-wall-the-big-one). Read it before
changing the torch install.

---

## TL;DR quick setup (fresh Orin, same JetPack 6.2)

```bash
# 1. Miniforge (conda) — aarch64
curl -fL -o /tmp/miniforge.sh \
  "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
bash /tmp/miniforge.sh -b -p $HOME/miniforge3
source ~/miniforge3/etc/profile.d/conda.sh

# 2. ROS 2 Jazzy + build tools, all in conda (RoboStack)
mamba create -y -n robosub -c robostack-jazzy -c conda-forge python=3.12 \
  ros-jazzy-ros-base ros-jazzy-robot-localization ros-jazzy-tf-transformations \
  ros-jazzy-nmea-msgs ros-jazzy-mavros-msgs ros-jazzy-vision-msgs ros-jazzy-cv-bridge \
  ros-jazzy-ros2cli colcon-common-extensions cmake ninja
conda activate robosub

# 3. GPU PyTorch — see §GPU PyTorch. Prefer the cached wheel:
pip install ~/robosub-wheels/torch-2.8.0-cp312-cp312-linux_aarch64.whl \
            ~/robosub-wheels/torchvision-*-cp312-cp312-linux_aarch64.whl
pip install --no-deps ultralytics spatialmath-python
pip install depthai            # OAK camera SDK (depthai-ros not on RoboStack)

# 4. Env activation: CUDA libs on the path + isolate DDS (see §Env activation)

# 5. Hardware drivers (IMU needs an out-of-tree kernel module)
bash drivers/xsens_mt/install.sh     # re-run after any kernel update

# 6. Build the ROS workspace (skip the sim package — it's Gazebo-Harmonic-only)
colcon build --packages-skip simulation
source install/setup.bash
```

---

## GPU PyTorch (the hard part)

> ⏳ **STATUS:** the from-source torch wheel is being built at the time of writing.
> The exact wheel filename and the confirmed `torch.cuda.is_available()` result are
> filled in once the build + GPU smoke-test pass. The *procedure* below is final.

**You cannot `pip install` a prebuilt GPU torch here.** RoboStack forces Python
3.11/3.12; the only prebuilt Jetson GPU-torch wheels are cp310 (glibc 2.35) or
cp312 (glibc **2.38**, i.e. Ubuntu 24.04). Neither fits py3.12-on-glibc-2.35.
Full reasoning in [§ Kinks](#the-gpu-torch--glibc-wall-the-big-one). So we **build
torch from source once** and cache the wheel.

### Build once, cache forever

The build runs in a throwaway `torchbuild` conda env (to keep the ROS env clean)
and drops the wheel in `~/robosub-wheels/`. Key settings (see the committed
`scripts/build_torch_jetson.sh`):

```bash
export TORCH_CUDA_ARCH_LIST="8.7"          # Orin
export USE_CUDA=1 USE_CUDNN=1 USE_CUSPARSELT=0
export CUDA_HOME=/usr/local/cuda-12.6
export CUDNN_INCLUDE_DIR=/usr/include  CUDNN_LIB_DIR=/usr/lib/aarch64-linux-gnu
export CC=/usr/bin/gcc CXX=/usr/bin/g++    # host gcc 11
export MAX_JOBS=8                          # 64 GB RAM → safe at full parallelism
export CMAKE_POLICY_VERSION_MINIMUM=3.5    # let cmake 4.x accept old submodule minimums
# git clone --branch v2.8.0 pytorch; submodules; pip install -r requirements.txt
python setup.py bdist_wheel                # -> dist/torch-2.8.0-cp312-cp312-linux_aarch64.whl
```

Then torchvision **0.23.0** is built the same way (must match torch 2.8.0), and both
wheels are copied to `~/robosub-wheels/`. **A new Orin on the same JetPack just
installs the cached wheels — no recompile.** Rebuild only when bumping torch or
JetPack.

### Verify GPU
```bash
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available(), torch.cuda.get_device_name(0))"
# expect: 2.8.0  12.6  True  Orin
```

---

## Env activation (CUDA path + DDS isolation)

ROS nodes need the host CUDA/tegra libs on `LD_LIBRARY_PATH`, and — because the
dev container uses `--net=host` — we set a distinct `ROS_DOMAIN_ID` so conda users
don't collide with anyone still on Docker. Put this in the env's activation hook so
it's automatic:

```bash
mkdir -p $CONDA_PREFIX/etc/conda/activate.d
cat > $CONDA_PREFIX/etc/conda/activate.d/robosub_env.sh <<'EOF'
export LD_LIBRARY_PATH=/usr/local/cuda-12.6/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:$LD_LIBRARY_PATH
export ROS_DOMAIN_ID=17          # pick a team convention; keep off Docker's default 0
EOF
```

---

## Hardware & udev

Physical devices are the one thing conda can't abstract — one process per serial
port. Stable symlinks come from udev rules on the host (`/etc/udev/rules.d/`,
`sudo udevadm control --reload`). **No passwordless sudo on this machine — a human
runs the `sudo` steps.**

| Device | Chip / bus | Node | Notes |
|---|---|---|---|
| **IMU** Xsens MTi-200 | native-USB `2639:0012` | `/dev/ttyUSB_imu` | **needs the `xsens_mt` kernel module** — see below |
| **DVL** Wayfinder | FTDI FT232R | `/dev/ttyUSB_dvl` | works with stock `ftdi_sio` |
| **Teensy** breakout | native-USB CDC | `/dev/ttyACM*` / `/dev/ttyUSB_teensy` | thruster/servo bridge |
| **Camera** OAK (DepthAI) | USB3 | — | use the `depthai` pip SDK (see §depthai) |

### IMU — install the `xsens_mt` driver (do NOT use ftdi_sio)
The MTi-200 is a **native-USB device, not an FTDI chip**. JetPack 6.2 ships with
`CONFIG_USB_SERIAL_XSENS_MT` **unset**, so nothing binds it. The correct fix is the
out-of-tree `xsens_mt` kernel module, packaged in `drivers/xsens_mt/`:

```bash
bash drivers/xsens_mt/install.sh     # builds the module + installs the udev rule
# re-run after every kernel/L4T update (the module is tied to the running kernel)
```
This yields a stable `/dev/ttyUSB_imu` and `/imu/data` at ~400 Hz. See
[§ IMU debug history](#imu-debug-history-the--32-saga) for why ftdi_sio is wrong.

### depthai (OAK camera)
`depthai-ros` is **not** packaged by RoboStack (0 builds), so install the DepthAI
**Python SDK** into the env (`pip install depthai`); the perception nodes import it
directly. If a node genuinely needs the `depthai_ros_driver` ROS package, build it
from source into the workspace.

---

## colcon build notes

```bash
conda activate robosub
colcon build --packages-skip simulation      # sim needs Gazebo Harmonic (Jazzy) — we're runtime-only
source install/setup.bash
```
- **`simulation`** is the only Jazzy-coupled package (depends on `gz-*` Harmonic
  libs). We run vehicle-runtime-only, so skip it. If you ever want sim in conda,
  that's separate work (RoboStack does ship `ros-jazzy-ros-gz`).
- Everything else uses only the stable ROS 2 core API (rclpy/rclcpp, common
  message packages, tf2, custom `msgs`/`xsens_mti_ros2_driver` via rosidl).

---

## Kinks & gotchas

### The GPU-torch ↔ glibc wall (the big one)
- **JetPack 6.2 = Ubuntu 22.04 = glibc 2.35.** ROS 2 Jazzy is built for Ubuntu
  24.04. That mismatch is the *entire reason the Docker container existed* — it
  provided a 24.04 userspace. **RoboStack** removes that need by shipping ROS as
  conda packages decoupled from the host OS.
- **RoboStack uses conda-forge's Python (3.11/3.12), not the distro's 3.10.**
  Verified: Humble pins `>=3.11,<3.12`, Jazzy `>=3.12,<3.13`. There is **no py3.10
  RoboStack** for any current distro on aarch64. So switching Jazzy→Humble does
  **not** help (and Humble's aarch64 `mavros-msgs` is py311-only, which has *no*
  Jetson torch at all).
- **Prebuilt Jetson GPU torch** (jetson-ai-lab `jp6`) is **cp310 only** for
  cu126/cu128; the sole **cp312** wheel (cu129) is built on 24.04 and fails to
  import on 22.04 with `GLIBC_2.38 not found`.
- **You cannot fake glibc 2.38 on 22.04.** conda can't supply a runtime glibc
  (`CONDA_OVERRIDE_GLIBC` only fools the solver → segfaults); `gcompat` is
  musl-only; patchelf/private-glibc breaks CUDA detection. (NVIDIA's own forum
  answer to this exact case: use a matching wheel or a 24.04 container.)
- **Conclusion → build torch from source** for py3.12 + CUDA 12.6 + glibc 2.35 +
  sm_87, and cache the wheel. That's the only path that is all-conda, keeps Jazzy,
  and keeps the GPU, without a reflash.
- **Future simplification:** conda-forge merged native **Tegra** pytorch support
  (PR #491: sm_87, CUDA 12.9, py3.10–3.14, sysroot 2.34 → would run on glibc 2.35).
  Not shipped as an installable build yet (as of 2026-06). When it lands, the
  source build can be replaced by a plain `conda install pytorch`. Re-check
  periodically.

### Smaller traps
- **pip `--extra-index-url` grabs the CPU torch from PyPI.** `pip install
  --extra-index-url <jetson> torch==2.8.0` installed `2.8.0+cpu` (`cuda_avail
  False`). Use the **direct wheel URL** (or the cached wheel) instead.
- **cmake 4.x + PyTorch source.** Set `CMAKE_POLICY_VERSION_MINIMUM=3.5` or old
  submodule `cmake_minimum_required`s error out.
- **`ninja` isn't in the base env / system cmake is old** — install both from
  conda-forge into the env.
- **No passwordless sudo.** Anything touching `/etc/udev`, kernel modules, or root
  sysfs must be run by a human.

---

## Dependency management (supersedes `DEPENDENCY_MANAGEMENT.md`)

The old Docker layering (system apt + a `/home/ros/env` venv + `--break-system-
packages` torch) is gone. In the conda world there is **one** environment:

- **ROS 2 + system-ish libs** → RoboStack conda packages (`ros-jazzy-*`) in `robosub`.
- **GPU stack** → the from-source torch/torchvision wheels + `ultralytics`,
  pip-installed into `robosub`. (Torch is deliberately **not** a colcon
  `install_requires`, so `colcon build` never pulls a CPU wheel over it.)
- **Camera** → `depthai` pip SDK.
- **Workspace packages** → `colcon build` into `install/`.

The old doc's core warning still holds: **perception's GPU torch must be the
Jetson/CUDA build, never a generic PyPI wheel** — the difference is CPU vs GPU
inference for the object detector.

---

## IMU debug history (the `-32` saga)

Kept so nobody re-runs a two-day dead end. Original symptom: `ftdi_sio` attached
`ttyUSB0/1` but every FTDI control transfer returned **`-32` (EPIPE/STALL)**; baud
never set; reads were garbage.

- **Misdiagnosis (wrong):** a tegra-xusb/kernel USB bug requiring an L4T reflash or
  a move to a true root USB port.
- **Actual root cause:** the MTi-200 (`2639:0012`) is a **native-USB device, not an
  FTDI chip** (`bDeviceClass=2`, if00=interrupt control, if01=bulk data). The debug
  session had force-bound it to `ftdi_sio` via `new_id`; ftdi_sio then sent FTDI
  *vendor* control requests to a non-FTDI device, which correctly **STALLs** them →
  the deterministic `-32`. Standard control transfers worked; only FTDI-vendor ones
  stalled — proof the device was rejecting FTDI commands, not a controller fault.
- **Fix (verified, `/imu/data` @ ~400 Hz):** build/install the in-kernel
  **`xsens_mt`** driver out-of-tree + a udev rule for `/dev/ttyUSB_imu`. All in
  `drivers/xsens_mt/` → `bash drivers/xsens_mt/install.sh` (re-run after kernel
  updates). The DVL's FT232R is a *real* FTDI part and keeps using `ftdi_sio`.

---

## What still lives in Docker (nothing, by design)

`.devcontainer/` and `sim_docker.sh` are retained only as a fallback/reference
until the conda path is fully validated on the vehicle; they can be removed once
this setup is confirmed in the water.
