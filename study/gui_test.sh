#!/usr/bin/env bash
# Try launching the Gazebo GUI under WSLg with software rendering.
pkill -9 -x px4; pkill -9 -x gz; sleep 2
export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=4.5 GALLIUM_DRIVER=llvmpipe
cd ~/PX4-Autopilot && (setsid make px4_sitl gz_x500 > /tmp/px4gui.log 2>&1 < /dev/null &)
sleep 75
grep -aiE "error|failed|ogre|opengl|render" /tmp/px4gui.log | head -8 | cut -c1-200
pgrep -fa "gz sim" | cut -c1-120 | head -3
