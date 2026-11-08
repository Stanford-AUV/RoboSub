# Simulation image on ROS 2 Humble, to match the sub.
#
# The sub runs Humble (RoboStack conda env, Python 3.10). The old .devcontainer runs the
# sim's ROS side on Jazzy, so in sim the stack ran on a different ROS version than on the
# vehicle. This image puts the sim's ROS side on Humble (Ubuntu 22.04, Python 3.10) with
# Gazebo Harmonic and Gazebo's Humble+Harmonic bridge (ros-humble-ros-gzharmonic).
#
# Build, from the repo root (also on Apple-silicon Macs):
#   docker build --platform linux/amd64 -t robosub-sim-humble -f docker/sim-humble.Dockerfile .
# Why amd64: Gazebo has published ros-humble-ros-gzharmonic for amd64 only. On an
# Apple-silicon Mac, Docker Desktop emulates amd64 (slower, fine for the ROS side).
#
# Run (mounts the repo at /ws; 4222 is the NATS port bridge_local.py on the Mac uses):
#   docker run -it --rm --platform linux/amd64 --name sim-humble -p 4222:4222 \
#     -v "$PWD":/ws robosub-sim-humble bash
#
# Only src/ and requirements.txt are sent to the build (see sim-humble.Dockerfile.dockerignore).

FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# 1. Gazebo's own package server: Gazebo Harmonic, its Python bindings (used by
#    bridge_docker.py), and ros_gz compiled for Humble + Harmonic.
#    ros-humble-ros-gzharmonic conflicts with Humble's default ros-humble-ros-gz*
#    (built for Gazebo Fortress); the base image has neither, so nothing to remove.
RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release gnupg \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      python3-gz-transport13 \
      python3-gz-msgs10 \
      python3-pip \
 && rm -rf /var/lib/apt/lists/*

# 2. NATS server: carries raw Gazebo messages between bridge_local.py (Mac) and
#    bridge_docker.py (this container). Only needed when Gazebo runs on the Mac.
#    Ubuntu 22.04 has no nats-server package (24.04 does, which is why the old Jazzy
#    container could apt-install it), so download the official release instead.
ARG NATS_VERSION=2.10.24
RUN curl -fsSL "https://github.com/nats-io/nats-server/releases/download/v${NATS_VERSION}/nats-server-v${NATS_VERSION}-linux-$(dpkg --print-architecture).tar.gz" \
      | tar -xz -C /tmp \
 && mv /tmp/nats-server-v${NATS_VERSION}-linux-*/nats-server /usr/local/bin/nats-server \
 && rm -rf /tmp/nats-server-v${NATS_VERSION}-linux-* \
 && nats-server --version

# 3. The repo's dependencies, baked in so every container starts ready.
#    rosdep reads every package.xml under src/. The gz-* keys are skipped because
#    step 1 already installed Harmonic, and rosdep would otherwise try Fortress packages.
#    -r keeps going past keys it cannot resolve; read its warnings.
#    NumPy stays below 2 (and below 1.25, the newest Ubuntu 22.04's SciPy 1.8 accepts):
#    Humble's apt packages (SciPy, cv_bridge) are compiled against NumPy 1.x and crash on
#    NumPy 2 ("numpy.dtype size changed"). The newest opencv-python requires NumPy >= 2,
#    so it is held at 4.11, the last release that allows NumPy 1.x.
#    setuptools is then put back to 58.2.0, the version commonly used with Humble's colcon:
#    a newer one gets pulled in by the requirements, needs a newer `packaging` than Ubuntu's,
#    and makes every Python package build fail ("unexpected keyword argument
#    'strip_trailing_zero'").
COPY src /tmp/deps/src
COPY requirements.txt /tmp/deps/requirements.txt
RUN apt-get update && rosdep update --rosdistro humble \
 && rosdep install -r --from-paths /tmp/deps/src --ignore-src -y --rosdistro humble \
      --skip-keys "gz-msgs10 gz-transport13 gz-common5 gz-gui8 gz-plugin2 gz-rendering8 gz-math8 gz-tools2 gz-cmake3 ros_gz ros_gz_bridge ros_gz_sim ros_gz_interfaces" \
 && python3 -m pip install --no-cache-dir -r /tmp/deps/requirements.txt "numpy<1.25" "opencv-python<4.12" \
 && python3 -m pip install --no-cache-dir "setuptools==58.2.0" \
 && rm -rf /var/lib/apt/lists/* /tmp/deps

# 4. Environment. GZ_VERSION tells ros_gz-based builds which Gazebo to use.
#    GZ_PARTITION keeps this container's Gazebo traffic separate (same value the
#    old devcontainer used).
ENV GZ_VERSION=harmonic
ENV GZ_PARTITION=127.0.0.1:ros
ENV GZ_SIM_RESOURCE_PATH=/ws/src/simulation/simulation/models
ENV PYTHONPATH=/ws/src/simulation
WORKDIR /ws
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc
