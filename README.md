# px4-offboard-timing-study

PX4 SITL offboard flight control from ROS 2 (Jazzy, Gazebo Harmonic x500), plus a measurement of how the quality of the
setpoint stream (rate, gaps, jitter, CPU load, simulation speed) affects offboard mode and tracking.

Start with `FINDINGS.md` for the results and their caveats. `RESEARCH_QUESTION.md` has the original question, the
literature gaps and future work.

## What is in here

| Path | Content |
|---|---|
| `ros2_ws/src/px4_offboard_study/` | ROS 2 package. `study_node`: arm, offboard, takeoff, hold, yaw turn, velocity square, land, with configurable stream degradation and a 100 Hz CSV log. `teleop_node`: joystick or keyboard, deadzone, limits, body frame to NED. `common.py`: PX4 QoS, publishers, commands. `mathutil.py`: pure maths with unit tests. |
| `study/` | `run_experiments.py` (experiment plans), `analyze.py` (tables, figures), `render_media.py` (screenshots, replay video), `teleop_check.py`, build and debug scripts. |
| `results/` | All logs: `final` (clean demo flights), `run2` (main sweeps), `run4` (simulation speed), `run5` (timeout parameter), `teleop`, `pilot_run`. Each has CSV logs, JSON metadata, `summary.csv` and `figures/`. |
| `setup_sudo.sh`, `build_px4.sh`, `study/sync_msgs.sh` | Environment setup in Ubuntu 24.04 WSL. |

## What the demo does

1. Streams `OffboardControlMode` and `TrajectorySetpoint` at 20 Hz before the mode switch (PX4 refuses offboard otherwise).
2. Switches to offboard, arms, climbs to 5 m, holds, turns 90 degrees.
3. Flies a 4 m velocity square (1 m/s legs), then lands.

PX4 topics are best effort and volatile; the nodes use the same QoS or nothing arrives.

## Run it (Ubuntu 24.04 in WSL2, ROS 2 Jazzy)

```bash
bash setup_sudo.sh          # apt packages and Gazebo (needs sudo)
bash build_px4.sh           # PX4 SITL, Micro XRCE-DDS agent, px4_msgs (30+ minutes)
bash study/sync_msgs.sh     # make px4_msgs match the PX4 source
# build the package
mkdir -p ~/study_ws && cp -r ros2_ws/src ~/study_ws/ && cd ~/study_ws
source /opt/ros/jazzy/setup.bash && source ~/px4_ws/install/setup.bash && colcon build
# one clean demo flight (starts simulator, agent, node; writes CSV and JSON)
cd study && python3 run_experiments.py final
```

Topic names carry message versions in this PX4 build; if they differ in yours, check `ros2 topic list -t` and edit
`common.py`.

## Showcase video (real Gazebo footage)

`px4_offboard_SHOWCASE_full_video.mp4` is built from three Gazebo flights (joystick, 0.6 s stream gap, 1.5 s stream gap)
recorded from the simulator window and composed with the flight logs: gamepad overlay, silence gauge against the 1.0 s
timeout, captions and three synced plots. The raw recordings are not in git (large); the logs are in `results/showcase_raw`.

```bash
# in Ubuntu WSL: world + texture, then simulator, agent and GUI on the AMD GPU
bash study/install_showcase.sh && WORLD=showcase bash study/gui_session.sh
bash study/gui_camera.sh -3 -3 1.6                  # follow camera offset (metres)
bash study/gui_act.sh teleop A_teleop               # or: gui_act.sh gap B_gap0.6 0.6
# in Windows: record the Gazebo window with ffmpeg ddagrab, then
python study/compose_video.py out.mp4
```

`GALLIUM_DRIVER=d3d12` (set by the scripts) renders on the GPU; without it WSL falls back to the CPU and the window runs
at about 4 frames per second.

## Tests

```bash
cd ros2_ws/src/px4_offboard_study && PYTHONPATH=. pytest -q test   # 9 tests, no ROS needed
```

CI runs the same on every push (`.github/workflows/test.yml`).

## Status and limits

- Everything is simulation. One vehicle model, one laptop, small samples (2-3 runs per condition).
- The WSL2 clock on this machine is unstable (wall clock stepped back about 2.5 s every 32 s). The nodes use steady-clock
  timers and PX4-anchored timestamps for that reason; see `FINDINGS.md` section 4.
- The large sweeps in `run2` and `run4` did not pin `COM_OF_LOSS_T`; `FINDINGS.md` section 3 explains what that means.
- Teleop was flown with scripted joystick messages, not a physical gamepad.
