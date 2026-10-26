#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash; source ~/px4_ws/install/setup.bash
ros2 topic list -t | grep -E "vehicle_status|vehicle_local_position|offboard_control_mode|trajectory_setpoint|vehicle_command"
