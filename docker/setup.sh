#!/usr/bin/env bash
set -eo pipefail

WS=/home/ubuntu/waveshare_ws

# Docker creates any missing parents of a bind mount as root, so the workspace
# root can end up unwritable for the container user.
if [ ! -w "$WS" ]; then
    sudo chown "$(id -u):$(id -g)" "$WS" "$WS/src"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
cd "$WS"
rosdep install --from-paths ./src --ignore-src -r -y
colcon build --symlink-install
