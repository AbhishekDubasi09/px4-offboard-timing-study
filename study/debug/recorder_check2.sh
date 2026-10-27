#!/usr/bin/env bash
S=/mnt/c/Users/abhib/px4_offboard_study/study
export GZ_IP=127.0.0.1
WORLD=showcase bash $S/gui_session.sh
echo "--- services with record/video:"; gz service -l 2>/dev/null | grep -i -E "record|video"
echo "--- all /gui services:"; gz service -l 2>/dev/null | grep "^/gui"
echo "--- config tail:"; tail -n 6 ~/gz_gui_minimal.config | cut -c1-140
pkill -x px4; pkill -x gz; pkill -x MicroXRCEAgent
