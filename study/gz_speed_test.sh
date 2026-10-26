#!/usr/bin/env bash
pkill -9 -x px4; pkill -9 -x gz; sleep 2
cd ~/PX4-Autopilot && (HEADLESS=1 setsid make px4_sitl gz_x500 > /tmp/px4m.log 2>&1 < /dev/null &)
B=~/PX4-Autopilot/build/px4_sitl_default/bin
until $B/px4-param show NAV_DLL_ACT >/dev/null 2>&1; do sleep 1; done
sleep 5
export GZ_IP=127.0.0.1
python3 - <<'PY'
import re, subprocess, time

def simt():
    o = subprocess.run('gz topic -e -n 1 -t /world/default/stats', shell=True, capture_output=True, text=True, timeout=10).stdout
    m = re.search(r'sim_time \{\s*sec: (\d+)\s*(?:nsec: (\d+))?', o)
    return int(m.group(1)) + int(m.group(2) or 0) / 1e9

def measure(label, secs=8):
    s0, w0 = simt(), time.monotonic()
    time.sleep(secs)
    s1, w1 = simt(), time.monotonic()
    print(f'{label:28s} sim/mono = {(s1 - s0) / (w1 - w0):.3f}', flush=True)

def setp(req):
    r = subprocess.run('gz service -s /world/default/set_physics --reqtype gz.msgs.Physics --reptype gz.msgs.Boolean '
                       f'--timeout 3000 --req "{req}"', shell=True, capture_output=True, text=True)
    print('  set', req, '->', r.stdout.strip().replace('\n', ' '))

measure('default')
setp('real_time_update_rate: 125'); measure('update_rate 125')
setp('real_time_update_rate: 250'); measure('update_rate 250')
setp('max_step_size: 0.002'); measure('step 0.002 (rate 250)')
setp('max_step_size: 0.004'); measure('step 0.004 (rate 250)')
setp('real_time_factor: 0.5'); measure('real_time_factor 0.5')
PY
pkill -9 -x px4; pkill -9 -x gz
