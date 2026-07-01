#!/usr/bin/env bash
# Build GPU PyTorch (+ torchvision) from source for Jetson AGX Orin.
#
# Why from source: RoboStack ROS 2 forces Python 3.11/3.12, but the only prebuilt
# Jetson GPU-torch wheels are cp310 (glibc 2.35) or cp312 (glibc 2.38 / Ubuntu 24.04).
# Neither fits py3.12-on-glibc-2.35 (JetPack 6.2 / Ubuntu 22.04). So we compile once
# and cache the wheels in ~/robosub-wheels/. See ORIN_SETUP.md for the full story.
#
# Verified on: AGX Orin 64GB, JetPack 6.2 (L4T R36.5, Ubuntu 22.04, glibc 2.35),
#              CUDA 12.6, cuDNN 9.3, gcc 11, Python 3.12.
#
# Usage:  bash scripts/build_torch_jetson.sh
set -eo pipefail

TORCH_VERSION="${TORCH_VERSION:-v2.8.0}"
VISION_VERSION="${VISION_VERSION:-v0.23.0}"
BUILD_ROOT="${BUILD_ROOT:-$HOME/torchbuild}"
WHEELS="${WHEELS:-$HOME/robosub-wheels}"
BUILD_ENV="${BUILD_ENV:-torchbuild}"

source ~/miniforge3/etc/profile.d/conda.sh

# Isolated build env so the ROS env's pins are never disturbed.
mamba env list | grep -q "^${BUILD_ENV} " || \
  mamba create -y -n "$BUILD_ENV" -c conda-forge python=3.12 cmake ninja numpy \
    pyyaml typing_extensions setuptools wheel cffi pip requests
conda activate "$BUILD_ENV"

mkdir -p "$BUILD_ROOT" "$WHEELS"

# --- CUDA / cuDNN / GPU arch (Orin = sm_87) ---
export CUDA_HOME=/usr/local/cuda-12.6
export CUDACXX="$CUDA_HOME/bin/nvcc"
export PATH="$CUDA_HOME/bin:$PATH"
export TORCH_CUDA_ARCH_LIST="8.7"
export USE_CUDA=1 USE_CUDNN=1 USE_CUSPARSELT=0 USE_CUFILE=0
export CUDNN_INCLUDE_DIR=/usr/include
export CUDNN_LIBRARY=/usr/lib/aarch64-linux-gnu/libcudnn.so
export CUDNN_LIB_DIR=/usr/lib/aarch64-linux-gnu
export CC=/usr/bin/gcc CXX=/usr/bin/g++
export MAX_JOBS="${MAX_JOBS:-8}"
export BUILD_TEST=0 USE_DISTRIBUTED=1
export CMAKE_POLICY_VERSION_MINIMUM=3.5   # cmake 4.x accepts old submodule minimums
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH:-}"

# --- PyTorch ---
cd "$BUILD_ROOT"
[ -d pytorch/.git ] || git clone --branch "$TORCH_VERSION" --depth 1 https://github.com/pytorch/pytorch.git
cd pytorch
git submodule sync && git submodule update --init --recursive
pip install -r requirements.txt
python setup.py bdist_wheel
cp dist/torch-*.whl "$WHEELS"/
pip install dist/torch-*.whl               # needed to build torchvision against it

# --- torchvision (must match the torch version) ---
cd "$BUILD_ROOT"
[ -d vision/.git ] || git clone --branch "$VISION_VERSION" --depth 1 https://github.com/pytorch/vision.git
cd vision
FORCE_CUDA=1 python setup.py bdist_wheel
cp dist/torchvision-*.whl "$WHEELS"/

echo "DONE — wheels in $WHEELS:"
ls -la "$WHEELS"
