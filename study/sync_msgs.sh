#!/usr/bin/env bash
# Make px4_msgs match the PX4 source tree that was built (a mismatch gives silent payload-size errors).
set -e
M=~/px4_ws/src/px4_msgs
cp ~/PX4-Autopilot/msg/*.msg ~/PX4-Autopilot/msg/versioned/*.msg $M/msg/
cp ~/PX4-Autopilot/srv/*.srv $M/srv/ 2>/dev/null || true
cp ~/PX4-Autopilot/srv/versioned/*.srv $M/srv/ 2>/dev/null || true
cd ~/px4_ws && source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
