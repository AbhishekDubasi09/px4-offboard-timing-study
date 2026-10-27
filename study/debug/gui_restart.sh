#!/usr/bin/env bash
# Restart only the Gazebo GUI with PX4's model path so the x500 meshes resolve.
pkill -f "gz sim -g"; sleep 2
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12} GZ_IP=127.0.0.1  # d3d12 = the AMD GPU through WSL; llvmpipe = CPU
export GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
(setsid gz sim -g --gui-config $HOME/gz_gui_minimal.config > /tmp/gzgui.log 2>&1 < /dev/null &)
sleep 25
bash /mnt/c/Users/abhib/px4_offboard_study/study/gui_camera.sh -3.2 -3.2 1.6 | tr -d '\n'; echo
