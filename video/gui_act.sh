#!/usr/bin/env bash
# Fly one recorded act against the running GUI session.
#   gui_act.sh teleop <name>            joystick flight (scripted analog sticks), log ~/act_<name>.csv
#   gui_act.sh gap <name> <gap_len_s>   autonomous flight with one setpoint-stream gap, log ~/act_<name>.csv
source /opt/ros/jazzy/setup.bash; source ~/px4_ws/install/setup.bash; source ~/study_ws/install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
case $1 in
  teleop)
    ros2 run px4_offboard_study teleop_node --ros-args -p source:=joy > ~/act_$2_node.log 2>&1 &
    NODE=$!
    python3 /mnt/c/Users/abhib/px4_offboard_study/experiments/teleop_driver.py ~/act_$2.csv
    kill $NODE 2>/dev/null ;;
  gap)
    ros2 run px4_offboard_study study_node --ros-args -p rate_hz:=20.0 -p gap_start:=3.5 -p gap_len:=$3 \
      -p log_path:=$HOME/act_$2.csv 2>&1 | grep -E "phase|left|result" | cut -c1-110 ;;
esac
echo "act $2 done"
