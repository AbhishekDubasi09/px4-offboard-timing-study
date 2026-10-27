#!/usr/bin/env bash
export GZ_IP=127.0.0.1
gz model --list 2>&1 | tail -6
gz model -m x500_0 --pose 2>&1 | head -8
tail -n 5 /tmp/gzgui.log | grep -av Warning | cut -c1-200
echo "gz gui procs: $(pgrep -fc 'gz sim -g')"
