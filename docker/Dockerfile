FROM osrf/ros:jazzy-desktop-full

# The container user. The Ubuntu 24.04 base image already has "ubuntu" (uid 1000), and with the
# default arguments that account is reused rather than created. The paths below and in
# .devcontainer/devcontainer.json and docker/setup.sh assume /home/ubuntu, so another USERNAME is
# not supported without changing them as well.
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN set -eux; \
    if ! id -u "$USERNAME" >/dev/null 2>&1; then \
        groupadd --gid "$USER_GID" "$USERNAME"; \
        useradd -s /bin/bash --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME"; \
    fi; \
    usermod -aG dialout "$USERNAME"; \
    id -nG "$USERNAME" | grep -qw dialout
# dialout: the serial port (/dev/ttyACM0 is root:dialout 0660). The base image's "ubuntu" happens
# to be in dialout already; the usermod above makes it explicit, and it is what a created user gets.
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME
USER $USERNAME

RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt update && sudo apt install -y \
    git \
    python3-pip \
    ros-${ROS_DISTRO}-diff-drive-controller

RUN rosdep update

# bam (Rhoban's Better Actuator Models) for actuator identification; kept on purpose.
WORKDIR /home/ubuntu/
RUN git clone https://github.com/Rhoban/bam.git
RUN pip install --break-system-packages ./bam

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "[ -f /home/ubuntu/waveshare_ws/install/setup.bash ] && source /home/ubuntu/waveshare_ws/install/setup.bash" >> ~/.bashrc
