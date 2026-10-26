# Findings

Measured on one laptop (Ryzen 7 7730U, WSL2 Ubuntu 24.04, ROS 2 Jazzy, PX4 SITL with headless Gazebo Harmonic x500).
All results are simulation only. This is an engineering characterisation, not a claim of new science.

## 1. The demo flight works

`results/final/demo_realtime.csv` and `demo.csv`: arm, offboard, climb to 5 m, hold, 90 degree yaw turn, velocity
square (1 m/s legs), land. The setpoint stream ran at 20 Hz (673 and 675 messages), offboard mode was never lost,
PX4 ran at 1.000x real time (measured from PX4's own clock). RMS velocity error on the square is about 0.48 m/s,
mostly the lag of the step response (`results/final/figures/fig0_demo_flight.png`).

## 2. What the experiments show

| Question | Result | Data |
|---|---|---|
| Steady stream rate | 1 Hz lost offboard (2 of 2 runs). 1.5, 2, 3, 5, 10, 20, 50 Hz kept it (14 of 14). | `run2`, rate runs |
| Tracking error vs rate | RMS velocity error rises as the rate falls: 0.47 m/s at 50 Hz, 0.48 at 10 Hz, 0.52 at 5 Hz, 0.53 at 3 Hz, 0.59 at 1.5-2 Hz. It degrades long before any failsafe. | `run2` |
| One gap in a 50 Hz stream | Gaps of 0.5 and 0.9 s tolerated (6 of 6). 1.0 s: 1 of 3 lost. 1.1, 1.2, 1.5 s: lost 9 of 9, about 1.0-1.2 s after the gap began. | `run2`, gap runs |
| Timing jitter, 10 Hz stream | Never lost offboard up to 0.2 s standard deviation (longest gap 0.81 s). Error rose from 0.47 to 0.61 m/s. | `run2` |
| CPU load on the companion | 12 and 24 stress threads: stream stayed at 20.0 Hz, longest gap 0.07 s, no loss. Error 0.48-0.54. | `run2` |
| Is the failsafe delay the PX4 parameter? | Yes. With `COM_OF_LOSS_T` set to 0.5 / 1.0 / 2.0 s at 1.0x speed, the loss came 0.57 / 1.05 / 2.0 s after the gap began (2 runs each, all agree within 0.05 s). | `run5` |
| Does the threshold follow PX4 time or wall time? | PX4 time. The delay converted to PX4 seconds is 0.5-0.6 s at both 1.0x and 0.5x speed (wall delay 0.55 s and 1.1 s). | `run4` |

What this adds to the PX4 documentation ("send faster than 2 Hz"): the real rule is a timeout on the **longest silence**
(`COM_OF_LOSS_T`, in PX4 time), not a rate. 2 Hz leaves a roughly 2x margin: a steady 1.5 Hz stream
(longest gap 0.67 s) was safe in these runs and 1 Hz was not. Control quality suffers well before the failsafe.

## 3. Things that must be said about these numbers

- **The parameter was not pinned in `run2` and `run4`.** The PX4 SITL start script rewrites timeout parameters whenever
  `PX4_SIM_SPEED_FACTOR` is set, and the simulator keeps parameters between runs. In `run4` the delay in PX4 time was
  0.5-0.6 s, so the timeout in force was most likely 0.5 s, not the default 1.0 s. The scaling with simulation speed
  is still valid, the absolute value is not the default. `run5` and `final` pin it (the runner now sets
  `COM_OF_LOSS_T 1.0` and clears saved parameters before every run).
- **`run2` has no measured real-time factor.** Its thresholds may be at 0.5x or 1.0x speed. Treat the 1.0 s boundary
  there as indicative only. A clean re-run with the pinned parameter is future work (about 1.5 hours).
- **Simulation speed varied between sessions** for the same step size (0.5x in some, 1.0x in others). The cause is
  unknown (host load or power state suspected). Every run since `run4` measures PX4's clock against the monotonic clock
  and stores it in its `.json`.
- **At quarter speed (0.25x) the picture is noisy.** Some failsafes fired about 2.8 s after a 1.0 s gap, after the stream
  had resumed, and a 2.5 s gap was retained once. A second failure mode (timestamp translation in the agent when the
  PX4 clock is slow) is likely but was not investigated.
- `run2` also contains six `slowgap_*` runs. Their half-speed setting did not take effect, so they are plain repeats and
  carry no speed information.
- `pilot_run` is the first sweep, made with an instrument that sent a nominal 50 Hz stream at only about 39 Hz and
  placed the gap where it could not affect tracking. It is kept for completeness only.
- Small samples (2-3 per condition), one vehicle model, simulation only, no real hardware.

## 4. Problems found and fixed on the way

| Symptom | Cause | Fix |
|---|---|---|
| No data from PX4 | Topic names carry message versions in this PX4 build (`vehicle_status_v5`, `vehicle_local_position_v1`, `vehicle_command_v1`) | Names in `common.py`; check with `ros2 topic list -t` |
| Silent payload-size errors, no data | `px4_msgs` was a week older than the PX4 source | `study/sync_msgs.sh` copies PX4's own definitions |
| Arming denied | No ground station in SITL | `NAV_DLL_ACT 0`, `COM_RC_IN_MODE 4` |
| Offboard dropped for no reason; node froze 2.5 s | WSL2 wall clock is stepped back by 2.48 s about every 32 s; `rclpy` timers follow the wall clock | Steady-clock timers |
| Offboard dropped after the clock step | Message timestamps came from the stepped wall clock and looked 2.5 s old | Timestamps anchored on the latest PX4 timestamp and advanced with the steady clock |
| Rates 20 % off | Scheduler waited for the next 10 ms tick | Schedule anchored on the previous due time |
| Wrong real-time factor | The XRCE agent maps PX4 time onto wall time | Read PX4's clock directly with `px4-listener` |

## 5. Joystick teleop check

`results/teleop/`: `teleop_node` with `source:=joy`, driven by scripted `sensor_msgs/Joy` messages (no physical
joystick was used). It takes off and holds under offboard control, then each stick command moves the drone as expected
(`study/teleop_check.py`, figure `figures/fig_teleop_check.png`):

| Stick | Duration | Result |
|---|---|---|
| forward (full) | 4 s | moved 6.6 m along the nose direction (east here, the x500 spawns facing 90 degrees), mean speed 1.7 m/s against 2.0 m/s limit |
| right (full) | 4 s | moved 6.7 m to the right of the nose (south), mean speed 1.7 m/s |
| up (0.6) / down (-0.6) | 3 s each | +1.58 m / -1.60 m altitude |
| yaw (full) | 4 s | turned 162 degrees (0.72 rad/s against 0.8 rad/s limit) |
| button 0 | | landed and disarmed |

This verifies the body-frame to NED conversion and the deadzone and limits end to end in simulation.
Not tested: a physical gamepad, and the keyboard variant (needs an interactive terminal).

## 6. Not done

- No comparison with another autopilot (ArduPilot is the obvious next one). See `RESEARCH_QUESTION.md`, future work.
