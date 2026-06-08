#!/bin/bash
set -e

TARGET=${1:-laptop}

if [ -z "$ROS_DISTRO" ]; then
    echo "[ERROR] ROS2 not sourced. Run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "[run.sh] Building workspace..."
colcon build --symlink-install

echo "[run.sh] Sourcing install/setup.bash..."
source install/setup.bash

echo "[run.sh] Launching mission: target=$TARGET"
ros2 launch capstone_bringup motion.launch.py target_class:=$TARGET
