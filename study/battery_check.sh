#!/usr/bin/env bash
pkill -9 -x px4; pkill -9 -x gz; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
for p in "COM_LOW_BAT_ACT 0" "SIM_BAT_DRAIN 100000" "SIM_BAT_MIN_PCT 90"; do $B/px4-param set $p 2>&1 | tail -1; done
sleep 20
$B/px4-param show -q COM_LOW_BAT_ACT SIM_BAT_DRAIN SIM_BAT_MIN_PCT BAT1_LOW_THR BAT1_CRIT_THR BAT1_EMERGEN_THR 2>&1 | grep -v "^$" | head -12
$B/px4-listener battery_status -n 1 2>&1 | grep -E "voltage_v|remaining|warning|connected" | head -6
$B/px4-listener failsafe_flags -n 1 2>&1 | grep -iE "battery|offboard" | head
