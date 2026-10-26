#!/usr/bin/env bash
pkill -9 -x px4; pkill -9 -x gz; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
$B/px4-param show -q COM_OF_LOSS_T COM_OBL_RC_ACT 2>&1 | grep -v "^$" | head -4
pkill -9 -x px4; pkill -9 -x gz
