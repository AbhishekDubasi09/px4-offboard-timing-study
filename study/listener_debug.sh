#!/usr/bin/env bash
pkill -9 -x px4; pkill -9 -x gz; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
sleep 10
$B/px4-listener vehicle_attitude -n 1 2>&1 | head -8
pkill -9 -x px4; pkill -9 -x gz
