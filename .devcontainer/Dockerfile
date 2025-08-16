
FROM althack/ros2:jazzy-full-2024-12-01

ENV DEBIAN_FRONTEND=noninteractive
ENV XAUTHORITY=/home/ros/.Xauthority
ENV QT_X11_NO_MITSHM=1

# Refresh ROS GPG key (fix EXPKEYSIG error)
RUN rm -f /etc/apt/sources.list.d/ros2.list && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -q -y --no-install-recommends \
    ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get -y install --no-install-recommends python3.12-venv python3-gz-transport13 ros-jazzy-robot-localization ros-jazzy-depthai-ros ros-jazzy-tf-transformations ros-jazzy-nmea-msgs ros-jazzy-mavros-msgs \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ENV USER ros
RUN id -u ros >/dev/null 2>&1 || useradd -m -u 1000 -s /bin/bash ros


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
# RUN ln -sf /host_home/.Xauthority /home/ros/.Xauthority

#ros-jazzy-mavros-msgs

RUN apt update && apt install -y python3-serial


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

# Set up auto-source of workspace for ros user
ARG WORKSPACE
# ARG WORKSPACE
COPY ${WORKSPACE}/requirements.txt /tmp/requirements.txt
RUN /home/ros/env/bin/pip install --upgrade pip && \
    /home/ros/env/bin/pip install -r /tmp/requirements.txt
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f /workspaces/RoboSub/ports.sh ]; then sudo bash /workspaces/RoboSub/ports.sh; fi" >> /home/ros/.bashrc
RUN echo "export GZ_PARTITION=127.0.0.1:ros" >> /home/ros/.bashrc

# ARG WORKSPACE=/workspaces/RoboSub
# ENV WORKSPACE=${WORKSPACE}

# # Create Python venv & upgrade pip
# RUN python3 -m venv /home/ros/env --system-site-packages --symlinks \
#  && /home/ros/env/bin/pip install --upgrade pip

# # Install your requirements.txt into the venv
# COPY requirements.txt /tmp/requirements.txt
# RUN /home/ros/env/bin/pip install -r /tmp/requirements.txt

# # Auto-source: activate venv, ROS overlay, ports.sh, and set partition
# RUN echo 'source /home/ros/env/bin/activate' >> /home/ros/.bashrc \
#  && echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc \
#  && echo "if [ -f ${WORKSPACE}/ports.sh ]; then bash ${WORKSPACE}/ports.sh; fi" >> /home/ros/.bashrc \
#  && echo 'export GZ_PARTITION=127.0.0.1:ros' >> /home/ros/.bashrc