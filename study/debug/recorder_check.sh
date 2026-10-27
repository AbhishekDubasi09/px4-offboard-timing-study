#!/usr/bin/env bash
export GZ_IP=127.0.0.1 GALLIUM_DRIVER=d3d12
echo "--- plugin library present?"; ls /usr/lib/x86_64-linux-gnu/gz-gui-8/plugins/ 2>/dev/null | grep -i -E "video|record" ; ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/gui 2>/dev/null | grep -i -E "video|record"
find /usr/lib -iname "*VideoRecorder*" 2>/dev/null | head -3
echo "--- GUI log mentions:"; sed 's/\x1b\[[0-9;]*m//g' /tmp/gzgui.log 2>/dev/null | grep -a -i -E "video|record|plugin.*(fail|not|could)" | head -5 | cut -c1-200
