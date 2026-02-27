FROM althack/ros2:jazzy-full-2024-12-01

ENV DEBIAN_FRONTEND=noninteractive

# Set CUDA environment variables
ENV CUDA_HOME=/usr/local/cuda-12
ENV PATH=/usr/local/cuda-12/bin:${PATH}
ENV LD_LIBRARY_PATH=/usr/local/cuda-12/lib64:/usr/lib/aarch64-linux-gnu/nvidia:/usr/lib/aarch64-linux-gnu/tegra:${LD_LIBRARY_PATH}

# Refresh ROS GPG key + source
RUN rm -f /usr/share/keyrings/ros-archive-keyring.gpg /etc/apt/sources.list.d/ros2.list && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    > /etc/apt/sources.list.d/ros2.list

# Install Gazebo repo key + list, then install
RUN wget -q https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends python3.12-venv python3-gz-transport13 ros-jazzy-robot-localization ros-jazzy-depthai-ros ros-jazzy-tf-transformations ros-jazzy-nmea-msgs ros-jazzy-mavros-msgs \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ENV USER=ros

# Setup IMU Utils
RUN apt-get update && apt-get install -y kmod \
    sharutils \
    file \
    libusb-1.0-0-dev \
    udev \ 
    usbutils

# Setup logging utils
RUN apt-get update && apt-get install -y less fzf

# Install nats-server
RUN apt-get update && apt-get install -y nats-server

# Install xquartz testing stuff
RUN apt-get update && apt-get install -y xauth x11-apps

# symbolic link .Xauthority
RUN ln -sf /host_home/.Xauthority /home/ros/.Xauthority

# Setup SSH server
RUN apt update && apt install -y openssh-server
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2222 >> /etc/ssh/sshd_config
RUN sed -i 's/\(^PermitEmptyPasswords\)/#\1/' /etc/ssh/sshd_config && echo PermitEmptyPasswords yes >> /etc/ssh/sshd_config
RUN passwd -d ${USER}
EXPOSE 2222
RUN service ssh restart

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN python3 -m venv /home/ros/env --system-site-packages --symlinks
# Base image - use an L4T image appropriate for JetPack 6.2
# FROM nvcr.io/nvidia/l4t-base:r36.4.0

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install cuDSS
RUN wget -q https://developer.download.nvidia.com/compute/cudss/0.5.0/local_installers/cudss-local-repo-ubuntu2204-0.5.0_0.5.0-1_arm64.deb && \
    dpkg -i cudss-local-repo-ubuntu2204-0.5.0_0.5.0-1_arm64.deb && \
    cp /var/cudss-local-repo-ubuntu2204-0.5.0/cudss-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y cudss && \
    rm -f cudss-local-repo-ubuntu2204-0.5.0_0.5.0-1_arm64.deb && \
    rm -rf /var/lib/apt/lists/*

# Install cuSPARSELt
RUN wget -q https://developer.download.nvidia.com/compute/cusparselt/0.8.1/local_installers/cusparselt-local-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    dpkg -i cusparselt-local-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    cp /var/cusparselt-local-repo-ubuntu2204-0.8.1/cusparselt-*-keyring.gpg /usr/share/keyrings/ && \
    apt-get update && \
    apt-get install -y cusparselt && \
    rm -f cusparselt-local-repo-ubuntu2204-0.8.1_0.8.1-1_arm64.deb && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libopenblas0 \
    && rm -rf /var/lib/apt/lists/*

# Set library path
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}

RUN pip install --break-system-packages \
    "https://pypi.jetson-ai-lab.io/jp6/cu129/+f/f36/0db283b1249b3/torch-2.8.0-cp312-cp312-linux_aarch64.whl" \
    "https://pypi.jetson-ai-lab.io/jp6/cu129/+f/255/341cb5beb4673/torchvision-0.23.0-cp312-cp312-linux_aarch64.whl#sha256=255341cb5beb4673f0f1a26061105623f17a2b32188478c68b3f2e1a897d49a2"
# Install ultralytics for system Python (allow dependency flexibility)
RUN python3 -m pip install --break-system-packages --no-cache-dir --no-deps ultralytics

# Set up auto-source of workspace for ros user
ARG WORKSPACE
ENV WORKSPACE="/workspaces/Robosub"
RUN echo "if [ -f ${WORKSPACE}/requirements.txt ]; then source /home/ros/env/bin/activate; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f /workspaces/RoboSub/ports.sh ]; then sudo bash /workspaces/RoboSub/ports.sh; fi" >> /home/ros/.bashrc
RUN echo "export GZ_PARTITION=127.0.0.1:ros" >> /home/ros/.bashrc