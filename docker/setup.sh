#!/usr/bin/env bash
set -eo pipefail

WS=/home/ubuntu/waveshare_ws

# Docker creates any missing parents of a bind mount as root, so the workspace
# root can end up unwritable for the container user.
if [ ! -w "$WS" ]; then
    sudo chown "$(id -u):$(id -g)" "$WS" "$WS/src"
fi

source "/opt/ros/$ROS_DISTRO/setup.bash"
echo "RT limits in this container: rtprio=$(ulimit -r) memlock=$(ulimit -l) (want 99 / unlimited)"
id -nG | grep -qw dialout || echo "WARNING: $(id -un) is not in dialout"
cd "$WS"
rosdep install --from-paths ./src --ignore-src -y
colcon build --symlink-install
