---
name: Dependency Management Analysis and PyTorch GPU Fix
overview: Analyze the complete dependency management system (Docker, venv, system Python, ROS2 packages, requirements.txt) and identify why PyTorch is using CPU instead of GPU. Document all dependency sources, installation locations, and resolution paths.
todos:
  - id: investigate-python-interpreter
    content: Determine which Python interpreter ROS2 nodes use when executed (system vs venv)
    status: completed
  - id: check-torch-installations
    content: Identify all torch installations and their sources (venv, system, colcon build)
    status: completed
  - id: verify-cuda-setup
    content: Verify CUDA libraries are accessible and torch can detect GPU
    status: completed
  - id: analyze-colcon-build
    content: Check if colcon build installs torch and from which source
    status: completed
  - id: fix-postcreate-command
    content: Fix postCreateCommand path and remove conflicting torch installation
    status: completed
  - id: resolve-setup-py-deps
    content: Decide how to handle torch in setup.py install_requires to avoid CPU-only installs
    status: completed
  - id: create-dependency-doc
    content: Create comprehensive documentation of dependency management system
    status: completed
isProject: false
---

# Dependency Management Analysis and PyTorch GPU Fix Plan

## Current Dependency Architecture

### 1. System-Level Dependencies (apt packages)

**Location**: Installed via `apt-get` in Dockerfile

- **ROS2 packages**: `/opt/ros/jazzy/` (system-wide ROS2 installation)
- **Python packages**: `/usr/lib/python3.12/site-packages/` (system Python 3.12)
- **ROS2 Python packages**: `/opt/ros/jazzy/lib/python3.12/site-packages/`
- **Examples**: `ros-jazzy-robot-localization`, `ros-jazzy-depthai-ros`, `python3-gz-transport13`

### 2. Virtual Environment (`/home/ros/env`)

**Location**: Created at line 66 in Dockerfile

- **Type**: `python3 -m venv /home/ros/env --system-site-packages --symlinks`
- **Key**: `--system-site-packages` means venv can see system packages
- **Packages installed**:
  - PyTorch (Jetson AI Lab, cu126): Lines 69-71
  - ultralytics: Line 74
  - Packages from `requirements.txt`: Installed via postCreateCommand (line 81)

### 3. System Python Packages

**Location**: Installed with `--break-system-packages` flag

- **PyTorch**: Lines 77-79 (Jetson AI Lab, cu126)
- **ultralytics**: Lines 82-83

### 4. ROS2 Workspace Packages (colcon build)

**Location**: `install/` directory after `colcon build`

- **Python packages**: Built via `setup.py` files in each ROS2 package
- **Perception package**: `src/perception/setup.py` lists `torch` and `ultralytics` in `install_requires` (lines 19-20)
- **Installation**: When colcon builds, it installs dependencies via pip into the ROS2 workspace's Python environment

### 5. Requirements.txt

**Location**: Root `requirements.txt`

- **Purpose**: General Python dependencies
- **Note**: Intentionally excludes `torch` (see comment lines 7-10) to avoid CPU-only wheels
- **Installed**: Via postCreateCommand into venv

## Critical Issues Identified

### Issue 1: Python Interpreter Mismatch

**Problem**: ROS2 Python nodes use system Python, not venv

- **Evidence**: All nodes use `#!/usr/bin/env python3` shebang
- **Impact**: When `ros2 run perception object_localizer` executes, it uses system Python (`/usr/bin/python3`), not venv Python (`/home/ros/env/bin/python3`)
- **Result**: System Python's torch installation is used, which may be CPU-only or wrong version

### Issue 2: Conflicting PyTorch Installations

**Problem**: Multiple PyTorch installations with different sources

1. **Dockerfile venv** (lines 69-71): Jetson AI Lab PyPI, cu126
2. **Dockerfile system** (lines 77-79): Jetson AI Lab PyPI, cu126
3. **postCreateCommand** (line 81): PyTorch official PyPI, cu124 (DIFFERENT SOURCE!)
4. **setup.py install_requires**: Will install torch when colcon builds (likely CPU-only from PyPI)

### Issue 3: Path Mismatch in postCreateCommand

**Problem**: Wrong workspace path

- **Line 81**: References `/workspaces/Robosub2/requirements.txt`
- **Actual**: Should be `/workspaces/Robosub/requirements.txt` (no "2")
- **Impact**: Requirements.txt may not be installed correctly

### Issue 4: ROS2 Package Dependencies Not Installed in System Python

**Problem**: `setup.py install_requires` dependencies install during colcon build

- **Location**: `src/perception/setup.py` line 19 lists `torch`
- **Behavior**: When `colcon build` runs, it installs torch via pip, likely pulling CPU-only version from default PyPI
- **Impact**: This may override or conflict with system Python's GPU-enabled torch

## Dependency Resolution Flow

### When ROS2 Node Executes:

1. `ros2 run perception object_localizer` is called
2. ROS2 looks up entry point in `install/perception/lib/perception/object_localizer`
3. Script has shebang `#!/usr/bin/env python3` → resolves to `/usr/bin/python3` (system Python)
4. System Python's `sys.path` includes:

- `/opt/ros/jazzy/lib/python3.12/site-packages/` (ROS2 packages)
- `/usr/lib/python3.12/site-packages/` (system packages)
- Workspace `install/` directory (if sourced)

1. `import torch` resolves to system Python's torch installation

### When colcon build Runs:

1. Reads `setup.py` files
2. Installs `install_requires` dependencies via pip
3. Uses whatever Python/pip is active (likely system Python)
4. Installs packages into workspace `install/` directory

## Root Cause Analysis

**Why PyTorch uses CPU mode:**

1. ROS2 nodes execute with system Python (`/usr/bin/python3`)
2. System Python's torch may be:

- CPU-only version installed by colcon build (from default PyPI)
- Wrong CUDA version (cu124 vs cu126)
- Missing CUDA libraries in container

1. Even if GPU-enabled torch is installed in system Python, CUDA runtime may not be accessible in container

## Files to Investigate/Modify

1. `**.devcontainer/Dockerfile` (lines 66-84): Venv and system Python torch installation
2. `**.devcontainer/devcontainer.json` (line 81): postCreateCommand with wrong path and conflicting torch install
3. `**src/perception/setup.py` (line 19): `install_requires` may pull CPU-only torch
4. `**requirements.txt`: Currently excludes torch (correct)
5. `**src/perception/perception/nodes/object_localizer.py`: Torch import and CUDA detection logic

## Recommended Solutions

### Solution 1: Ensure System Python Uses GPU-Enabled Torch

- Remove conflicting torch installations
- Ensure only Jetson AI Lab PyPI torch (cu126) is installed in system Python
- Verify CUDA libraries are accessible in container

### Solution 2: Fix postCreateCommand

- Correct path from `/workspaces/Robosub2/` to `/workspaces/Robosub/`
- Remove conflicting torch installation (already installed in Dockerfile)
- Or ensure it uses same source as Dockerfile

### Solution 3: Handle setup.py install_requires

- Option A: Remove `torch` from `install_requires` (rely on system installation)
- Option B: Specify exact torch version/source in `install_requires`
- Option C: Use `--no-deps` when building ROS2 packages and install torch separately

### Solution 4: Verify CUDA Access

- Ensure `--runtime=nvidia` or `--gpus all` is working
- Verify CUDA libraries are mounted correctly
- Check `torch.cuda.is_available()` returns True

## Investigation Steps

1. Check which Python interpreter ROS2 nodes actually use
2. Verify which torch installation is being imported
3. Check CUDA availability in container
4. Verify CUDA library paths are correct
5. Check if colcon build is installing CPU-only torch
6. Verify postCreateCommand path and execution
