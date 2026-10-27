#!/usr/bin/env bash
# Measure PX4's own clock (and Gazebo's sim time) against the monotonic clock, no XRCE agent involved.
pkill -9 -x px4; pkill -9 -x gz; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
sleep 10
export GZ_IP=127.0.0.1
python3 - <<'PY'
import re, subprocess, time
B = '/home/abhishek/PX4-Autopilot/build/px4_sitl_default/bin'

def px4t():
    o = subprocess.run(f'{B}/px4-listener vehicle_attitude -n 1', shell=True, capture_output=True, text=True, timeout=10); o = o.stdout + o.stderr
    return int(re.search(r'timestamp: (\d+)', o).group(1)) / 1e6

def gzt():
    o = subprocess.run('gz topic -e -n 1 -t /world/default/stats', shell=True, capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'sim_time \{\s*sec: (\d+)\s*(?:nsec: (\d+))?', o)
    return int(m.group(1)) + int(m.group(2) or 0) / 1e9

def measure(label, secs=20):
    a = (px4t(), gzt(), time.monotonic())
    time.sleep(secs)
    b = (px4t(), gzt(), time.monotonic())
    dm = b[2] - a[2]
    print(f'{label:24s} px4/mono = {(b[0]-a[0])/dm:.3f}   gz/mono = {(b[1]-a[1])/dm:.3f}', flush=True)

measure('default')
measure('default (again)')
PY
pkill -9 -x px4; pkill -9 -x gz
