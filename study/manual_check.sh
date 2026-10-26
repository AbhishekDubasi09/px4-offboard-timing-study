#!/usr/bin/env bash
# Bring up SITL + agent and print what ROS 2 sees. Debug helper.
pkill -9 -x px4; pkill -9 -x gz; pkill -9 -x MicroXRCEAgent; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
(setsid ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 > /tmp/agm.log 2>&1 < /dev/null &)
sleep 30
source /opt/ros/jazzy/setup.bash; source ~/px4_ws/install/setup.bash
ros2 topic info -v /fmu/out/vehicle_status_v5 2>&1 | grep -E "Reliab|Durab|Node name|Publisher count" | head
timeout 8 ros2 topic echo --once --qos-reliability best_effort --qos-durability volatile /fmu/out/vehicle_status_v5 2>&1 | head -4
timeout 8 ros2 topic echo --once --qos-reliability best_effort --qos-durability transient_local /fmu/out/vehicle_status_v5 2>&1 | head -4
tail -3 /tmp/agm.log | cut -c1-160
