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

# PyTorch with CUDA for Jetson (JP6 CUDA 12.6) - using Jetson AI Lab PyPI
RUN /home/ros/env/bin/pip install --no-cache-dir \
    --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    torch torchvision torchaudio

# Install ultralytics (YOLO) in venv
RUN /home/ros/env/bin/pip install --no-cache-dir ultralytics

# Install PyTorch for system Python (Jetson version from AI Lab PyPI)
RUN python3 -m pip install --break-system-packages --no-cache-dir \
    --extra-index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    torch torchvision torchaudio

# Install ultralytics for system Python (allow dependency flexibility)
RUN python3 -m pip install --break-system-packages --no-cache-dir ultralytics || \
    python3 -m pip install --break-system-packages --no-cache-dir --no-deps ultralytics

# Set up auto-source of workspace for ros user
ARG WORKSPACE
ENV WORKSPACE="/workspaces/Robosub2"
RUN echo "if [ -f ${WORKSPACE}/requirements.txt ]; then source /home/ros/env/bin/activate; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f /workspaces/RoboSub2/ports.sh ]; then sudo bash /workspaces/RoboSub2/ports.sh; fi" >> /home/ros/.bashrc
RUN echo "export GZ_PARTITION=127.0.0.1:ros" >> /home/ros/.bashrc