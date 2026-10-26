"""Run one or more offboard flights in PX4 SITL (headless Gazebo x500). Run inside Ubuntu WSL.

Usage: python3 run_experiments.py demo
       python3 run_experiments.py sweep_rate | sweep_gap | sweep_jitter
Each run starts a fresh simulator, the XRCE agent and study_node, and writes <name>.csv/.json/.log.
"""
import json
import os
import signal
import subprocess
import sys
import time

HOME = os.path.expanduser('~')
PX4 = f'{HOME}/PX4-Autopilot'
AGENT = f'{HOME}/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'
WS = f'{HOME}/px4_ws/install/setup.bash'
STUDY_WS = f'{HOME}/study_ws/install/setup.bash'
OUT = os.environ.get('STUDY_OUT', f'{HOME}/study_results')
ENV = 'export FASTDDS_BUILTIN_TRANSPORTS=UDPv4 && source /opt/ros/jazzy/setup.bash && source ' + WS + ' && source ' + STUDY_WS


def sh(cmd, log, **kw):
    return subprocess.Popen(['bash', '-lc', cmd], stdout=open(log, 'w'), stderr=subprocess.STDOUT,
                            preexec_fn=os.setsid, **kw)


def kill(p):
    try:
        os.killpg(os.getpgid(p.pid), signal.SIGINT)
        p.wait(timeout=10)
    except Exception:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGKILL)
        except Exception:
            pass


def cleanup():
    subprocess.run('pkill -9 -x px4; pkill -9 -x gz; pkill -9 -x MicroXRCEAgent; pkill -9 -f ros2.run.px4_offboard_study; rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* 2>/dev/null; sleep 2', shell=True)


def run_one(name, params, speed=1.0, load=0):
    os.makedirs(OUT, exist_ok=True)
    cleanup()
    csv_path = f'{OUT}/{name}.csv'
    px4 = sh(f'cd {PX4} && PX4_SIM_SPEED_FACTOR={speed} HEADLESS=1 make px4_sitl gz_x500', f'{OUT}/{name}_px4.log')
    agent = sh(f'FASTDDS_BUILTIN_TRANSPORTS=UDPv4 {AGENT} udp4 -p 8888', f'{OUT}/{name}_agent.log')
    stress = sh(f'stress-ng --cpu {load} --timeout 300s', f'{OUT}/{name}_stress.log') if load else None
    pxparam = f'{PX4}/build/px4_sitl_default/bin/px4-param'
    for _ in range(90):  # wait until the PX4 server answers
        if subprocess.run(f'{pxparam} show NAV_DLL_ACT', shell=True, capture_output=True).returncode == 0:
            break
        time.sleep(1)
    # SITL has no GCS or RC, and the simulated battery drains while waiting; none of these may trigger a
    # failsafe, otherwise it would be confused with the offboard-loss failsafe under study.
    for prm in ('NAV_DLL_ACT 0', 'COM_RC_IN_MODE 4', 'COM_LOW_BAT_ACT 0', 'SIM_BAT_DRAIN 100000', 'SIM_BAT_MIN_PCT 90'):
        subprocess.run(f'{pxparam} set {prm}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(15)  # EKF alignment
    args = ' '.join(f'-p {k}:={v}' for k, v in {**params, 'log_path': csv_path}.items())
    node = sh(f'{ENV} && ros2 run px4_offboard_study study_node --ros-args {args}', f'{OUT}/{name}_node.log')
    try:
        node.wait(timeout=240)
    except subprocess.TimeoutExpired:
        kill(node)
    for p in (stress, agent, px4):
        if p:
            kill(p)
    cleanup()
    res = f'{OUT}/{name}.json'
    return json.load(open(res)) if os.path.exists(res) else None


PLANS = {
    'demo': [('demo', {'rate_hz': 20.0})],
    'sweep_rate': [(f'rate_{r}hz', {'rate_hz': r}) for r in (1.0, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0)],
    'sweep_gap': [(f'gap_{g}s', {'rate_hz': 50.0, 'gap_start': 2.0, 'gap_len': g})
                  for g in (0.1, 0.3, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0)],
    'sweep_jitter': [(f'jitter_{j}', {'rate_hz': 10.0, 'jitter_std': j}) for j in (0.0, 0.02, 0.05, 0.1, 0.2)],
}

if __name__ == '__main__':
    for name, params in PLANS[sys.argv[1]]:
        print(name, run_one(name, params), flush=True)
