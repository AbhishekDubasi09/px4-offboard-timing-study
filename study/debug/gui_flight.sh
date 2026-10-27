#!/usr/bin/env bash
# Fly the demo against the running GUI session and note PX4's clock speed during the flight.
source /opt/ros/jazzy/setup.bash; source ~/px4_ws/install/setup.bash; source ~/study_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
B=~/PX4-Autopilot/build/px4_sitl_default/bin
t() { $B/px4-listener vehicle_attitude -n 1 2>&1 | grep -a -m1 "timestamp:" | awk '{print $2/1e6}'; }
a=$(t); m0=$(date +%s.%N)
ros2 run px4_offboard_study study_node --ros-args -p rate_hz:=20.0 -p log_path:=$HOME/gui_flight.csv 2>&1 | grep -E "phase|left|result" | cut -c1-120
b=$(t); m1=$(date +%s.%N)
echo "rtf_px4_over_wall: $(awk -v a=$a -v b=$b -v m0=$m0 -v m1=$m1 'BEGIN{printf "%.3f", (b-a)/(m1-m0)}')"
