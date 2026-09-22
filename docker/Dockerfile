FROM osrf/ros:jazzy-desktop-full

# change user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
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

WORKDIR /home/ubuntu/
RUN git clone https://github.com/Rhoban/bam.git
RUN pip install --break-system-packages ./bam

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "[ -f /home/ubuntu/waveshare_ws/install/setup.bash ] && source /home/ubuntu/waveshare_ws/install/setup.bash" >> ~/.bashrc
