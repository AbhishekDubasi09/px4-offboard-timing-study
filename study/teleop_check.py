"""Fly teleop_node in SITL with scripted joystick input and save the log. Run inside Ubuntu WSL.

Usage: python3 teleop_check.py <out_dir>
"""
import os
import subprocess
import sys
import time

from run_experiments import AGENT, ENV, PX4, cleanup, kill, sh

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
cleanup()
px4 = sh(f'cd {PX4} && HEADLESS=1 make px4_sitl gz_x500', f'{out}/teleop_px4.log')
agent = sh(f'FASTDDS_BUILTIN_TRANSPORTS=UDPv4 {AGENT} udp4 -p 8888', f'{out}/teleop_agent.log')
pxparam = f'{PX4}/build/px4_sitl_default/bin/px4-param'
for _ in range(90):
    if subprocess.run(f'{pxparam} show NAV_DLL_ACT', shell=True, capture_output=True).returncode == 0:
        break
    time.sleep(1)
for prm in ('NAV_DLL_ACT 0', 'COM_RC_IN_MODE 4', 'COM_LOW_BAT_ACT 0', 'SIM_BAT_DRAIN 100000', 'SIM_BAT_MIN_PCT 90',
            'COM_OF_LOSS_T 1.0'):
    subprocess.run(f'{pxparam} set {prm}', shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
time.sleep(15)
node = sh(f'{ENV} && ros2 run px4_offboard_study teleop_node --ros-args -p source:=joy', f'{out}/teleop_node.log')
drv = sh(f'{ENV} && python3 {os.path.dirname(os.path.abspath(__file__))}/teleop_driver.py {out}/teleop_flight.csv',
         f'{out}/teleop_driver.log')
try:
    drv.wait(timeout=180)
except subprocess.TimeoutExpired:
    kill(drv)
for p in (node, agent, px4):
    kill(p)
cleanup()
print('done', out)
