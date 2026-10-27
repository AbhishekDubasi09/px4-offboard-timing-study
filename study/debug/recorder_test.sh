#!/usr/bin/env bash
# Test Gazebo's built-in video recorder (captures the 3D view directly, not the desktop).
S=/mnt/c/Users/abhib/px4_offboard_study/study
bash $S/install_showcase.sh >/dev/null
export GZ_IP=127.0.0.1
WORLD=showcase bash $S/gui_session.sh
rm -f /tmp/rec_test.mp4
echo "--- start:"; gz service -s /gui/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 3000 --req 'start: true, format: "mp4", save_filename: "/tmp/rec_test.mp4"' 2>&1 | tr -d '\n'; echo
sleep 8
echo "--- stop:"; gz service -s /gui/record_video --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 --req 'stop: true' 2>&1 | tr -d '\n'; echo
sleep 4
ls -la /tmp/rec_test.mp4 2>&1
pkill -x px4; pkill -x gz; pkill -x MicroXRCEAgent
