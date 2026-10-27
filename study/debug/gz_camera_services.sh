#!/usr/bin/env bash
# Which GUI camera services does this Gazebo expose, and is the python transport binding available?
export GZ_IP=127.0.0.1
pkill -9 -x px4; pkill -9 -f "gz sim"; sleep 1
export GALLIUM_DRIVER=d3d12 GZ_SIM_RESOURCE_PATH=$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds
cd ~/PX4-Autopilot && (WORLD=showcase PX4_GZ_WORLD=showcase HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4cam.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
(setsid gz sim -g --gui-config $HOME/gz_gui_minimal.config > /tmp/gzgui2.log 2>&1 < /dev/null &)
sleep 25
echo "--- gui services:"; gz service -l 2>/dev/null | grep -E "^/gui" | head -20
echo "--- follow info:"; gz service -i -s /gui/follow/offset 2>&1 | head -6; gz service -i -s /gui/move_to/pose 2>&1 | head -6
echo "--- python bindings:"; python3 -c "from gz.transport13 import Node; from gz.msgs10.vector3d_pb2 import Vector3d; print('gz.transport13 python OK')" 2>&1 | tail -1
pkill -9 -x px4; pkill -9 -f "gz sim"
