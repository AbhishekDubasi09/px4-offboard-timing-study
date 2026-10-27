"""Run one or more offboard flights in PX4 SITL (headless Gazebo x500). Run inside Ubuntu WSL.

Usage: python3 run_experiments.py demo
       python3 run_experiments.py sweep_rate | sweep_gap | sweep_jitter
Each run starts a fresh simulator, the XRCE agent and study_node, and writes <name>.csv/.json/.log.
"""
import json
import os
import re
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
    subprocess.run('pkill -9 -x px4; pkill -9 -x gz; pkill -9 -x MicroXRCEAgent; pkill -9 -f ros2.run.px4_offboard_study; rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps* ' + PX4 + '/build/px4_sitl_default/rootfs/parameters* 2>/dev/null; sleep 2', shell=True)


def px4_time():
    """PX4's own clock in seconds (read straight from the autopilot, not through the XRCE agent, whose
    timesync maps PX4 time onto wall time and would hide the simulation speed)."""
    r = subprocess.run(f'{PX4}/build/px4_sitl_default/bin/px4-listener vehicle_attitude -n 1', shell=True,
                       capture_output=True, text=True, timeout=15)
    m = re.search(r'timestamp: (\d+)', r.stdout + r.stderr)
    return int(m.group(1)) / 1e6 if m else None


def run_one(name, params, step=None, load=0, px4_params=()):
    """step: Gazebo max_step_size in s (default world 0.004). The sim is CPU-bound at about 125 steps/s here,
    so real-time factor = 125 * step: 0.002 -> 0.25, 0.004 -> 0.5, 0.008 -> 1.0."""
    os.makedirs(OUT, exist_ok=True)
    cleanup()
    csv_path = f'{OUT}/{name}.csv'
    px4 = sh(f'cd {PX4} && HEADLESS=1 make px4_sitl gz_x500', f'{OUT}/{name}_px4.log')
    agent = sh(f'FASTDDS_BUILTIN_TRANSPORTS=UDPv4 {AGENT} udp4 -p 8888', f'{OUT}/{name}_agent.log')
    stress = sh(f'stress-ng --cpu {load} --timeout 300s', f'{OUT}/{name}_stress.log') if load else None
    pxparam = f'{PX4}/build/px4_sitl_default/bin/px4-param'
    for _ in range(90):  # wait until the PX4 server answers
        if subprocess.run(f'{pxparam} show NAV_DLL_ACT', shell=True, capture_output=True).returncode == 0:
            break
        time.sleep(1)
    # SITL has no GCS or RC, and the simulated battery drains while waiting; none of these may trigger a
    # failsafe, otherwise it would be confused with the offboard-loss failsafe under study.
    for prm in ('NAV_DLL_ACT 0', 'COM_RC_IN_MODE 4', 'COM_LOW_BAT_ACT 0', 'SIM_BAT_DRAIN 100000', 'SIM_BAT_MIN_PCT 90', 'COM_OF_LOSS_T 1.0', *px4_params):
        subprocess.run(f'{pxparam} set {prm}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if step:  # PX4 runs in lockstep with the Gazebo clock, so the step size sets PX4's speed too
        for _ in range(60):
            r = subprocess.run('GZ_IP=127.0.0.1 gz service -s /world/default/set_physics --reqtype gz.msgs.Physics '
                               f'--reptype gz.msgs.Boolean --timeout 2000 --req "max_step_size: {step}"',
                               shell=True, capture_output=True, text=True)
            if 'true' in r.stdout:
                break
            time.sleep(1)
    time.sleep(15)  # EKF alignment
    a = (px4_time(), time.monotonic())
    args = ' '.join(f'-p {k}:={v}' for k, v in {**params, 'log_path': csv_path}.items())
    node = sh(f'{ENV} && ros2 run px4_offboard_study study_node --ros-args {args}', f'{OUT}/{name}_node.log')
    try:
        node.wait(timeout=420)
    except subprocess.TimeoutExpired:
        kill(node)
    b = (px4_time(), time.monotonic())
    for p in (stress, agent, px4):
        if p:
            kill(p)
    cleanup()
    res = f'{OUT}/{name}.json'
    if not os.path.exists(res):
        return None
    meta = json.load(open(res))
    if a[0] is not None and b[0] is not None:
        meta['rtf_px4_mono'] = (b[0] - a[0]) / (b[1] - a[1])  # PX4 seconds per monotonic second
    meta['step'] = step or 0.004
    meta['load_threads'] = load
    meta['px4_params'] = list(px4_params)
    json.dump(meta, open(res, 'w'), indent=2)
    return meta


# Each entry: (run name, node parameters, run_one keyword options). The gap starts 3.5 s into the track
# phase so that it spans the command change at 4.0 s: a lost command is then visible in tracking error.
GAP_START = 3.5


def gap_runs(lengths, seeds, step=None, tag='gap'):
    return [(f'{tag}_{g}s_s{s}', {'rate_hz': 50.0, 'gap_start': GAP_START, 'gap_len': g, 'seed': s},
             {'step': step}) for g in lengths for s in seeds]


PLANS = {
    'demo': [('demo', {'rate_hz': 20.0}, {})],
    'rate': [(f'rate_{r}hz_s{s}', {'rate_hz': r, 'seed': s}, {})
             for r in (1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0) for s in (0, 1)],
    'gap': gap_runs((0.5, 0.9, 1.0, 1.1, 1.2, 1.5), (0, 1, 2)),
    'jitter': [(f'jitter_{j}_s0', {'rate_hz': 10.0, 'jitter_std': j}, {}) for j in (0.0, 0.02, 0.05, 0.1, 0.2)],
    'load': [(f'load{n}_s{s}', {'rate_hz': 20.0, 'seed': s}, {'load': n}) for n in (12, 24) for s in (0, 1)],
    # Simulation speed (real-time factor about 125 * step): does the failsafe threshold move with PX4 time?
    'timeout_param': [(f'oloss{t}_s{s}', {'rate_hz': 50.0, 'gap_start': GAP_START, 'gap_len': 3.0, 'seed': s},
                       {'step': 0.008, 'px4_params': (f'COM_OF_LOSS_T {t}',)}) for t in (0.5, 1.0, 2.0) for s in (0, 1)],
    'final': [('demo', {'rate_hz': 20.0}, {}), ('demo_realtime', {'rate_hz': 20.0}, {'step': 0.008})],
    'speed_check': [(f'speedcheck_step{st}', {'rate_hz': 20.0}, {'step': st}) for st in (0.002, 0.008)],
    'rtf': gap_runs((0.7, 1.0, 1.3, 2.5), (0, 1), step=0.002, tag='rtf0.25_gap')
    + gap_runs((0.7, 1.0, 1.3, 2.5), (0, 1), step=0.008, tag='rtf1.0_gap')
    + gap_runs((0.7, 1.0, 1.3, 2.5), (0, 1), step=0.004, tag='rtf0.5_gap'),
}

if __name__ == '__main__':
    for name, params, opts in PLANS[sys.argv[1]]:
        print(name, run_one(name, params, **opts), flush=True)
