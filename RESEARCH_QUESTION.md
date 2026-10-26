# PX4 offboard control: research question (2026-10-26)

## What the literature covers
- PX4 docs: offboard needs a >2 Hz proof-of-life stream (OffboardControlMode + setpoint) started before the mode switch; loss for longer than `COM_OF_LOSS_T` triggers failsafe. Rule is stated, not measured.
- PX4 docs: uXRCE-DDS topics are best-effort/volatile; default-QoS ROS 2 subscribers silently get nothing. Documented, not quantified.
- Forum/issues: 0.5-1 s apparent control delay, failsafe right after the mode switch, blocking sleeps starving the executor, uXRCE-DDS timesync resets when real-time factor (RTF) < 1 (workaround `UXRCE_DDS_SYNCT 0`). All anecdotal.
- ROS 2 timing (arXiv 2101.02074, 2408.08440, 2508.10413, 2601.10722 survey, ROSRT): executor/DDS latency and jitter are well studied, but for generic nodes, not for a flight-controller setpoint stream.
- UAV dependability survey (arXiv 2506.16786): says delay/loss destabilise control, with no PX4 offboard numbers.

## Gaps
1. No measured mapping from setpoint-stream quality (rate, jitter, burst gap) to (a) offboard retention and (b) tracking error. The "2 Hz" rule is folklore with unknown margin.
2. Which statistic matters (mean rate vs. maximum inter-arrival gap vs. gap distribution) is untested.
3. Companion-side causes (rclpy executor, CPU load, blocking callbacks, QoS mismatch) are never separated from link causes.
4. Clock-domain issue: PX4 timeouts run on PX4 time (lockstep sim time), ROS timers on wall time. If RTF != 1, a "wall-clock Hz" threshold is not transferable. Reported thresholds in sim are unreliable unless this is controlled.

## Research question
**In PX4 SITL offboard control, which property of the companion's setpoint stream (mean rate, jitter, or maximum inter-arrival gap) best predicts loss of offboard mode and degradation of position/velocity tracking, how large is the safety margin of the documented 2 Hz rule, and does the answer change with the companion implementation (rclpy timer, rclpy under CPU load, rclcpp) and with simulation RTF?**

## Hypotheses
- H1: Offboard loss is governed by the max gap vs `COM_OF_LOSS_T` (default 1.0 s), not mean rate. A stream at 50 Hz mean with one 1.2 s stall fails; 3 Hz steady does not.
- H2: Tracking error degrades well before failsafe (gaps ~0.2-0.5 s), so "2 Hz" is sufficient for retention but not for performance.
- H3: Thresholds measured in wall-clock Hz shift with RTF; expressing them in PX4 time removes the shift.
- H4: Under CPU load, rclpy produces heavy-tailed gaps that cross the threshold even at high nominal rate.

## Experiments (all in this repo)
- E1 rate sweep: steady 1, 2, 3, 5, 10, 20, 50 Hz; record offboard retained, RMS tracking error on a velocity square/circle.
- E2 gap injection: 50 Hz stream with one dropped window of 0.1...2.0 s; find the critical gap.
- E3 jitter: fixed mean rate, increasing Gaussian/heavy-tail jitter.
- E4 companion: rclpy vs rclcpp, idle vs `stress-ng` load.
- E5 RTF: repeat E1/E2 at RTF 1 and 0.5 (`PX4_SIM_SPEED_FACTOR`), report in wall vs PX4 time.
- Metrics: time-to-failsafe, mode-retained flag, RMS/max tracking error, observed inter-arrival gap stats at PX4 (from `vehicle_control_mode`/timestamps).

## Build (demo = the project in the image)
ROS 2 Jazzy + px4_msgs + `px4_offboard_study` package: `study_node` (arm, takeoff, hold, turn, velocity square, land, with
configurable stream degradation and a per-tick log), `teleop_node` (joystick or keyboard, deadzone, limits, body-frame to NED),
and `study/run_experiments.py` for the sweeps. The experiments as actually run are described in `FINDINGS.md`.

## Status after the first round (2026-10-27)
Short version of what was learned is in `FINDINGS.md`. Honest assessment: "the threshold is the documented timeout" is close
to expected, so the question as written is an engineering characterisation, not new science. The parts with the most
promise: (a) the failsafe delay lives in PX4 time, so thresholds measured in wall-clock Hz do not transfer between
simulation speeds; (b) at slow simulation speed a second failure mode appears (failsafe after the stream resumed);
(c) control quality degrades well before the failsafe.

## Future work (pick one when revisiting the question)
1. **Second autopilot.** Send identical degraded command streams to ArduPilot (GUIDED mode over MAVLink) and compare the
   safe limits with PX4. I believe ArduPilot's guided-velocity timeout is longer (about 3 s) but this is unverified.
2. **Clean re-run.** Re-run the rate, gap, jitter and load sweeps with `COM_OF_LOSS_T` pinned and the real-time factor
   measured per run (runner already does this), at 1.0x speed. About 1.5 hours.
3. **Quarter-speed failure mode.** Find out why failsafes fire after the stream resumed when PX4's clock is slow
   (suspect XRCE timesync). Compare with `UXRCE_DDS_SYNCT 0`.
4. **Real hardware.** Repeat the gap test on a real flight controller (HITL or a bench setup) to see which results survive.
5. **Other degradations.** Packet reordering, partial loss (drop heartbeat but keep setpoints and the reverse),
   and rclcpp versus rclpy under real-time scheduling.
