#!/usr/bin/env bash
# Builds PX4 SITL (gz x500), Micro XRCE-DDS Agent and px4_msgs. Run as normal user in Ubuntu-24.04 WSL.
set -e
cd ~
[ -d PX4-Autopilot ] || git clone --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
pip install --user --break-system-packages -r PX4-Autopilot/Tools/setup/requirements.txt
[ -d Micro-XRCE-DDS-Agent ] || git clone --depth 1 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
(cd Micro-XRCE-DDS-Agent && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j3)
mkdir -p ~/px4_ws/src && cd ~/px4_ws/src
[ -d px4_msgs ] || git clone --depth 1 https://github.com/PX4/px4_msgs.git
cd ~/px4_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select px4_msgs
cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl_default -j3
echo BUILD_ALL_DONE
