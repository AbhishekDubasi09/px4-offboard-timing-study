#!/usr/bin/env bash
# Start PX4 SITL (server), the XRCE agent and the Gazebo GUI (software rendering) and leave them running.
pkill -9 -x px4; pkill -9 -x gz; pkill -9 -x MicroXRCEAgent; pkill -9 -f "gz sim"; rm -f /dev/shm/fastrtps* ~/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters*; sleep 3
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12} GZ_IP=127.0.0.1  # d3d12 = the AMD GPU through WSL; llvmpipe = CPU
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
cd ~/PX4-Autopilot && (PX4_GZ_WORLD=${WORLD:-default} HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4gui.log 2>&1 < /dev/null &)
(FASTDDS_BUILTIN_TRANSPORTS=UDPv4 setsid ~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888 > /tmp/agentgui.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
for p in "NAV_DLL_ACT 0" "COM_RC_IN_MODE 4" "COM_LOW_BAT_ACT 0" "SIM_BAT_DRAIN 100000" "SIM_BAT_MIN_PCT 90" "COM_OF_LOSS_T 1.0"; do $B/px4-param set $p >/dev/null 2>&1; done
(setsid gz sim -g --gui-config $HOME/gz_gui_minimal.config > /tmp/gzgui.log 2>&1 < /dev/null &)
sleep 40
echo "gui up: $(pgrep -f 'gz sim -g' | wc -l)"
