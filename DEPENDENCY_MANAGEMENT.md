# Dependency Management Documentation

This document provides a comprehensive overview of how dependencies are managed in the RoboSub project, including system packages, Python packages, ROS2 packages, and GPU-specific dependencies.

## Overview

The project uses a multi-layered dependency management system:
1. **System-level packages** (apt) - ROS2, system libraries
2. **System Python packages** - GPU-enabled PyTorch, ultralytics
3. **Virtual environment** (`/home/ros/env`) - Development dependencies
4. **ROS2 workspace packages** - Built via colcon
5. **Requirements.txt** - General Python dependencies

## Dependency Layers

### 1. System-Level Dependencies (APT)

**Location**: Installed via `apt-get` in `.devcontainer/Dockerfile`

**Installation Points**:
- Lines 11-15: ROS2 repository setup
- Lines 18-22: Gazebo repository setup
- Lines 27-31: Core ROS2 and Python packages
- Lines 37-42: Hardware utilities (USB, udev)
- Lines 45-48: Development tools

**Key Packages**:
- `ros-jazzy-*` - ROS2 packages installed system-wide
- `python3.12-venv` - Python virtual environment support
- `python3-gz-transport13` - Gazebo transport libraries
- System libraries: `libusb-1.0-0-dev`, `udev`, `usbutils`

**Installation Paths**:
- ROS2 packages: `/opt/ros/jazzy/`
- ROS2 Python packages: `/opt/ros/jazzy/lib/python3.12/site-packages/`
- System Python packages: `/usr/lib/python3.12/site-packages/`

### 2. Virtual Environment (`/home/ros/env`)

**Location**: Created at line 66 in `.devcontainer/Dockerfile`

**Configuration**:
```bash
python3 -m venv /home/ros/env --system-site-packages --symlinks
```

**Key Features**:
- `--system-site-packages`: Venv can access system-installed packages
- `--symlinks`: Uses symlinks instead of copies for efficiency

**Packages Installed in Venv**:
- **PyTorch** (lines 69-71): Jetson AI Lab PyPI, CUDA 12.6
  ```bash
  /home/ros/env/bin/pip install --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch torchvision torchaudio
  ```
- **ultralytics** (line 74): YOLO library
- **Requirements.txt packages** (via postCreateCommand): General dependencies

**Purpose**: 
- Used by VSCode Python extension (configured in `.vscode/settings.json`)
- Development tools (black formatter, etc.)
- Interactive Python development

**Note**: ROS2 nodes do NOT use this venv by default - they use system Python.

### 3. System Python Packages

**Location**: Installed with `--break-system-packages` flag

**Packages Installed**:
- **PyTorch** (lines 77-79): Jetson AI Lab PyPI, CUDA 12.6
  ```bash
  python3 -m pip install --break-system-packages --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 torch torchvision torchaudio
  ```
- **ultralytics** (lines 82-83): YOLO library

**Purpose**: 
- Used by ROS2 Python nodes (which execute with system Python)
- Must be GPU-enabled for object_localizer to work

**Critical**: This is the torch installation that ROS2 nodes actually use!

### 4. ROS2 Workspace Packages (colcon build)

**Location**: `install/` directory after `colcon build`

**Build Process**:
1. `colcon build` reads `setup.py` files in each ROS2 package
2. Installs `install_requires` dependencies via pip
3. Creates entry points in `install/<package>/lib/<package>/`

**Python Packages**:
- Each ROS2 Python package has a `setup.py` file
- `install_requires` lists Python dependencies
- Entry points defined in `entry_points['console_scripts']`

**Important Notes**:
- **Perception package**: `torch` and `ultralytics` are NOT in `install_requires` to avoid installing CPU-only versions
- These packages rely on system Python's GPU-enabled installations instead
- When colcon builds, it uses system Python by default

**Entry Point Execution**:
- ROS2 nodes use shebang `#!/usr/bin/env python3`
- This resolves to system Python (`/usr/bin/python3`), NOT venv Python
- System Python's `sys.path` includes:
  - `/opt/ros/jazzy/lib/python3.12/site-packages/` (ROS2 packages)
  - `/usr/lib/python3.12/site-packages/` (system packages)
  - Workspace `install/` directory (if sourced)

### 5. Requirements.txt

**Location**: Root `requirements.txt`

**Purpose**: General Python dependencies for development

**Key Features**:
- Intentionally excludes `torch` (see comment lines 7-10)
- Prevents pulling CPU-only wheels from default PyPI
- Installed into venv via postCreateCommand

**Installation**: 
- Executed in `.devcontainer/devcontainer.json` postCreateCommand
- Installed into `/home/ros/env` venv

## PyTorch GPU Configuration

### Installation Sources

The project uses **Jetson AI Lab PyPI** for GPU-enabled PyTorch:

**Source**: `https://pypi.jetson-ai-lab.io/jp6/cu126`
- **JP6**: Jetson Platform 6 (Orin)
- **cu126**: CUDA 12.6

**Installations**:
1. **Venv** (Dockerfile lines 69-71): For development tools
2. **System Python** (Dockerfile lines 77-79): For ROS2 nodes ⚠️ **CRITICAL**

### Why System Python Installation is Critical

ROS2 Python nodes execute with system Python (`/usr/bin/python3`), not venv Python. Therefore:

1. System Python must have GPU-enabled torch installed
2. CUDA libraries must be accessible in container
3. Container must be run with GPU support (`--runtime=nvidia` or `--gpus all`)

### CUDA Configuration

**Environment Variables** (set in Dockerfile and devcontainer.json):
```bash
CUDA_HOME=/usr/local/cuda-12
PATH=/usr/local/cuda-12/bin:${PATH}
LD_LIBRARY_PATH=/usr/local/cuda-12/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra
```

**Container Runtime**: 
- `--runtime=nvidia` (devcontainer.json line 16)
- Or `--gpus all` for newer nvidia-container-toolkit

**Volume Mounts** (devcontainer.json lines 42-45):
- CUDA libraries mounted from host
- NVIDIA and Tegra libraries mounted

## Dependency Resolution Flow

### When ROS2 Node Executes

1. User runs: `ros2 run perception object_localizer`
2. ROS2 looks up entry point: `install/perception/lib/perception/object_localizer`
3. Script shebang: `#!/usr/bin/env python3` → resolves to `/usr/bin/python3` (system Python)
4. Python imports:
   - `import torch` → resolves to `/usr/lib/python3.12/site-packages/torch/`
   - `import ultralytics` → resolves to `/usr/lib/python3.12/site-packages/ultralytics/`
5. Torch checks CUDA availability: `torch.cuda.is_available()`

### When colcon build Runs

1. Reads `setup.py` files in each package
2. Installs `install_requires` dependencies via pip
3. Uses system Python by default
4. Installs packages into workspace `install/` directory
5. Creates entry point scripts with `#!/usr/bin/env python3` shebang

### When Requirements.txt is Installed

1. Executed via postCreateCommand in devcontainer.json
2. Installs into venv: `/home/ros/env/bin/pip install -r requirements.txt`
3. Used by VSCode Python extension and development tools

## Common Issues and Solutions

### Issue: PyTorch Using CPU Instead of GPU

**Symptoms**:
- `torch.cuda.is_available()` returns `False`
- Object localizer falls back to CPU inference

**Possible Causes**:
1. **Wrong torch installation**: CPU-only version installed instead of GPU version
2. **CUDA libraries not accessible**: Container not running with GPU support
3. **Wrong CUDA version**: Mismatch between torch build and CUDA runtime

**Solutions**:
1. Verify system Python has Jetson AI Lab torch installed:
   ```bash
   python3 -c "import torch; print(torch.__file__); print(torch.version.cuda)"
   ```
2. Check container GPU access:
   ```bash
   nvidia-smi  # Should work inside container
   ```
3. Verify CUDA libraries:
   ```bash
   ls -la /usr/local/cuda-12/lib64/libcudart.so*
   ```
4. Check torch CUDA availability:
   ```bash
   python3 -c "import torch; print(torch.cuda.is_available())"
   ```

### Issue: Conflicting PyTorch Installations

**Symptoms**: Multiple torch versions installed, wrong one being used

**Solution**: 
- Ensure only Jetson AI Lab PyPI torch is installed in system Python
- Remove torch from `setup.py install_requires` (already done)
- Remove conflicting installations from postCreateCommand (already fixed)

### Issue: Requirements.txt Not Installing

**Symptoms**: Dependencies from requirements.txt missing

**Solution**: 
- Check postCreateCommand path is correct (fixed: `/workspaces/Robosub/requirements.txt`)
- Verify postCreateCommand executed successfully

## Best Practices

1. **GPU Dependencies**: Always install GPU-enabled packages (torch, etc.) in system Python, not via colcon build
2. **Development Dependencies**: Install in venv for VSCode and development tools
3. **ROS2 Package Dependencies**: Only include dependencies that can be installed from standard PyPI
4. **Avoid Conflicts**: Don't install the same package in multiple locations with different sources
5. **Documentation**: Keep this file updated when dependency management changes

## File Reference

**Key Files**:
- `.devcontainer/Dockerfile`: System and Python package installation
- `.devcontainer/devcontainer.json`: Container configuration and postCreateCommand
- `requirements.txt`: General Python dependencies
- `src/*/setup.py`: ROS2 package dependencies
- `src/*/package.xml`: ROS2 package metadata

## Verification Commands

**Check Python interpreter used by ROS2**:
```bash
ros2 run perception object_localizer --help 2>&1 | head -1
# Check the shebang in install/perception/lib/perception/object_localizer
```

**Check torch installation**:
```bash
python3 -c "import torch; print('Torch version:', torch.__version__); print('CUDA available:', torch.cuda.is_available()); print('CUDA version:', torch.version.cuda if torch.cuda.is_available() else 'N/A'); print('Torch location:', torch.__file__)"
```

**Check venv torch**:
```bash
/home/ros/env/bin/python3 -c "import torch; print('Venv torch:', torch.__file__)"
```

**List installed packages**:
```bash
# System Python
python3 -m pip list | grep torch

# Venv
/home/ros/env/bin/pip list | grep torch
```

**Check CUDA libraries**:
```bash
ls -la /usr/local/cuda-12/lib64/libcudart.so*
ldconfig -p | grep cuda
```
