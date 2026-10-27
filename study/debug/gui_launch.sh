#!/usr/bin/env bash
export LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=4.5 GALLIUM_DRIVER=llvmpipe GZ_IP=127.0.0.1
echo DISPLAY=$DISPLAY WAYLAND=$WAYLAND_DISPLAY
pgrep -fa "gz sim" | cut -c1-100
(setsid gz sim -g > /tmp/gzgui.log 2>&1 < /dev/null &)
sleep 40
pgrep -fa "gz sim -g" | cut -c1-100
tail -n 8 /tmp/gzgui.log | cut -c1-200
