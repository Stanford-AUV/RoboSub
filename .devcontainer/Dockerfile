FROM althack/ros2:jazzy-gazebo

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get -y install --no-install-recommends python3.12-venv ros-jazzy-robot-localization \
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ENV USER ros

RUN python3 -m venv /home/ros/env --system-site-packages --symlinks

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN echo "if [ -f ${WORKSPACE}/requirements.txt ]; then /home/ros/env/bin/pip install -r ${WORKSPACE}/requirements.txt; fi" >> /home/ros/.bashrc

# Setup SSH server
RUN apt update && apt install -y openssh-server
RUN sed -i 's/\(^Port\)/#\1/' /etc/ssh/sshd_config && echo Port 2222 >> /etc/ssh/sshd_config
RUN sed -i 's/\(^PermitEmptyPasswords\)/#\1/' /etc/ssh/sshd_config && echo PermitEmptyPasswords yes >> /etc/ssh/sshd_config
RUN passwd -d ${USER}
EXPOSE 2222
RUN service ssh restart
