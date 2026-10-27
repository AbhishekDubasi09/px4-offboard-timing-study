# PX4 offboard control and the cost of silence

The full story of this project, written for my own use in interviews and for pasting into a Claude conversation.
Code, logs and the two videos are in this repository (private): `px4-offboard-timing-study`.

## Note for Claude (memory)

If this document is pasted into a conversation, I want Claude to save a memory of this project (type: project) so that
later sessions know it without me explaining it again. Suggested memory text:

> Project: px4-offboard-timing-study (private GitHub repo AbhishekDubasi09/px4-offboard-timing-study, local folder
> C:\Users\abhib\px4_offboard_study). A ROS 2 Jazzy node flies a simulated x500 quadcopter in PX4 SITL with Gazebo
> Harmonic through micro-XRCE-DDS in offboard mode. On top of that sits a measurement study of how the quality of the
> setpoint stream (rate, gaps, jitter, CPU load, simulation speed) affects offboard mode and tracking, and two kinds of
> video made from real Gazebo footage (an analysis cut with black and white backgrounds, and a cinematic cut, silent or with a drone sound synthesised from the flight log).
> Built 26 to 27 October 2026 with Claude Code as a pair programmer, for the Stark C-UAS Edge Software working student
> application. Main result: PX4 leaves offboard mode when the silence between setpoints exceeds COM_OF_LOSS_T, counted in
> PX4 time. With the timeout set to 0.5, 1.0 and 2.0 s, offboard was lost 0.57, 1.05 and 2.0 s after the stream stopped.
> Caveats to always keep: simulation only, 2 to 3 runs per condition, some sweeps ran before the timeout parameter was
> pinned (FINDINGS.md section 3), the joystick was flown with scripted messages not a physical pad, and the claim is an
> engineering characterisation, not new science. Deliverables: results/videos (white analysis video, cinematic video),
> PROJECT_STORY.md, FINDINGS.md.

## 1. What this is, in one paragraph

A drone has two computers. The flight controller (PX4) keeps it in the air. A companion computer (here a ROS 2 node)
decides what to do and sends commands. PX4 only accepts those commands in a special mode called offboard mode, and only
while a steady stream of messages keeps arriving. If the stream goes silent for longer than a timeout, PX4 assumes the
companion computer has died and takes the drone back. I built the whole chain in simulation, from a ROS 2 node to a flying
quadcopter in Gazebo, then measured exactly how much silence PX4 tolerates and what happens to control quality before it
gives up. I also made the evidence visible: real Gazebo footage synced to the flight logs.

```
joystick or script
        |
  ROS 2 node (Python, rclpy)
        |  /fmu/in/offboard_control_mode  /fmu/in/trajectory_setpoint  /fmu/in/vehicle_command_v1
        v
  micro-XRCE-DDS agent  <---- UDP 8888 ---->  PX4 SITL (uxrce_dds_client)  <-- lockstep clock -->  Gazebo Harmonic, x500
        ^                                              |
        |  /fmu/out/vehicle_local_position_v1  /fmu/out/vehicle_status_v5
        +----------------------------------------------+
```

## 2. Why I built it

- **For the job.** Stark's C-UAS Edge Software working student role is about edge software, telemetry, latency, tests in
  simulation and then field tests on UAV hardware. My project sheet said "a flight controller only accepts commands from a
  companion computer when the link is set up exactly right, I wanted to see this end to end". I wanted that sentence to be
  backed by logs, not just words.
- **Because the documentation leaves a gap.** PX4 says to send offboard setpoints faster than 2 Hz and to start the stream
  before switching mode. It states the rule, but nothing I found measures how much margin that rule has, which property of
  the stream matters (average rate, jitter or the longest gap), or how the answer depends on the companion's timing. Forum
  posts describe odd delays and failsafes but give no numbers.
- **Because latency was already my habit.** In my driver monitoring project a constant 55 ms delay hid behind a good
  accuracy figure. I wanted to apply the same habit, log a timestamp at every stage and look for the gap, to a flight
  controller link.

How it was built, stated plainly: I developed it with Claude Code as an AI pair programmer. I set the goals, chose the
research direction and the look of the videos, reviewed every result and caught problems (for example a pole hiding the
drone in my own footage). Every number in this repository is backed by a log in `results/`, and the problems I hit are
written down in section 5 so I can explain them.

## 3. What I built

| Part | What it does |
|---|---|
| `ros2_ws/src/px4_offboard_study` | ROS 2 package. `study_node` flies the sequence arm, offboard, climb to 5 m, hold, yaw turn, velocity square, land, with a configurable stream (rate, jitter, one dropped window) and a 100 Hz log. `teleop_node` flies by joystick or keyboard: deadzone, limits, body frame to NED. `common.py` holds the PX4 QoS, publishers and commands. `mathutil.py` has the pure maths with 14 unit tests. |
| `experiments/run_experiments.py` | Experiment runner. Starts a fresh simulator, the agent and the node for every run, pins the PX4 parameters, measures PX4's own clock speed, writes CSV and JSON. |
| `experiments/analyze.py` | Tables and figures from the logs. |
| `video/camera_director.py` | A scripted camera operator for the Gazebo window: tilt-up, chase, orbit, top-down, push-in, at 30 updates per second. |
| `video/compose_final.py` | The analysis video (palettes A black, D white): footage, gamepad, silence counter, captions and three plots, synced to the logs. |
| `video/compose_cinema.py` | The cinematic video: 2.39:1 letterbox, a restrained grade, slow motion, and (in one version) a drone sound synthesised from the flight log, `video/cine_audio.py`. |
| `video/make_showcase_world.py` | Generates the Gazebo world (checkerboard, flat markers, far-away blocks). |
| Tests and CI | 14 pytest tests on the stick mapping and conversions, run on every push. |

## 4. What I found (the short version)

Details and the honest caveats are in `FINDINGS.md`.

- PX4 does not need a fast stream. It needs a stream that is never silent for longer than `COM_OF_LOSS_T` (default 1.0 s),
  counted in PX4 time. With the timeout set to 0.5, 1.0 and 2.0 s, PX4 left offboard 0.57, 1.05 and 2.0 s after the stream
  stopped (2 runs each, within 0.05 s of each other).
- A steady 1.5 Hz stream kept offboard in all 14 runs from 1.5 to 50 Hz. A steady 1 Hz stream lost it in both runs, because
  its gap of 1.0 s equals the timeout. So the documented 2 Hz leaves roughly a 2x margin.
- Single gaps in a 50 Hz stream: 0.5 and 0.9 s tolerated (6 of 6), 1.0 s split (1 of 3 lost), 1.1 s and longer lost
  offboard mode in 9 of 9 runs.
- Control quality suffers long before any failsafe. The RMS velocity error rose from 0.47 m/s at 50 Hz to 0.59 m/s at
  1.5 to 2 Hz.
- Timing jitter up to 0.2 s standard deviation, and CPU load of 12 and 24 stress threads on the companion, did not cause
  a loss in these runs.
- The timeout runs in PX4 time, so a threshold measured in wall-clock hertz moves with the simulation speed.

## 5. The deep technical challenges and how I solved them

Each one has the same shape: what I saw, how I found the cause, the cause, the fix, and the lesson.

### 5.1 Nothing arrived from PX4 (message names and message layouts)
- **Saw:** the node subscribed to `/fmu/out/vehicle_status` and received nothing.
- **Found by:** `ros2 topic list -t` and reading the PX4 source. In this PX4 build the topics carry a message version
  (`vehicle_status_v5`, `vehicle_local_position_v1`, `vehicle_command_v1`).
- **Second layer:** after fixing the names there were silent errors such as "payload size 96 bytes is larger than 91".
  My `px4_msgs` package was a week older than the PX4 source I had compiled, so the two sides disagreed on the layout.
- **Fix:** names set in one place, and `setup/sync_msgs.sh` copies PX4's own message definitions into `px4_msgs` and rebuilds it.
- **Lesson:** two repositories that look the same can disagree on a wire format, and the failure is silence, not an error.

### 5.2 PX4 refused to arm, then flew away
- **Saw:** "Arming denied", and later a return-to-launch right after takeoff.
- **Cause:** SITL has no ground station and no remote control, and its simulated battery drains while it waits.
- **Fix:** pin `NAV_DLL_ACT 0`, `COM_RC_IN_MODE 4`, `COM_LOW_BAT_ACT 0`, `SIM_BAT_DRAIN`, `SIM_BAT_MIN_PCT`.
  This matters beyond convenience: any other failsafe would be confused with the offboard-loss failsafe I was studying.

### 5.3 The node froze for 2.5 seconds at random (a clock that jumps)
- **Saw:** offboard mode dropped for no reason, always with a gap of exactly 2.49 s in my 100 Hz log.
- **Found by:** Python's `faulthandler` showed the thread idle in the executor, not blocked in my code. A 60 second test
  comparing the wall clock with the monotonic clock showed the WSL2 wall clock stepping back by 2.48 s about every 32 s.
  A stopwatch on the Windows side showed the cause: the VM's monotonic clock runs about 9 percent fast, and Hyper-V time
  sync corrects the wall clock in steps.
- **Cause:** `rclpy` timers follow the wall clock by default, so each backward step froze them.
- **Fix:** timers on the steady (monotonic) clock.

### 5.4 The same clock fault, a second way (message timestamps)
- **Saw:** with the timers fixed, offboard mode still dropped, now exactly after a clock step.
- **Found by:** logging my own message timestamp next to the latest PX4 timestamp. They agreed to 20 ms until the step,
  then differed by minus 2.465 s.
- **Cause:** PX4 judges the stream by the timestamp inside each message. Mine came from the stepped wall clock, so after
  the step every message looked 2.5 s old, older than the 1.0 s timeout.
- **Fix:** stamp messages in PX4's own time: take the latest PX4 timestamp as an anchor and advance it with the steady clock.

### 5.5 The simulation was secretly running at half speed (measuring the right clock)
- **Saw:** a "real-time factor" column of 1.000 in every run, while the simulator felt slow.
- **Found by:** measuring PX4's own clock with `px4-listener` against the monotonic clock, and Gazebo's sim time the same
  way. Both said 0.50. The XRCE agent's time sync maps PX4 time onto wall time, so any timestamp that went through the
  agent could never reveal the speed.
- **Fix:** the runner reads PX4's clock directly before and after every flight and stores the result in the run's JSON.
  The speed is controlled through Gazebo's step size (0.002, 0.004, 0.008 s gave 0.25, 0.5, 1.0). I also found that the
  speed varied between sessions for identical settings (cause not identified), which is why it is measured per run.
- **Lesson:** a metric computed from a pipeline that hides the thing you want to measure is worse than no metric.

### 5.6 Where the failsafe threshold really lives (time domains)
- **Saw:** the failsafe came about 1.1 s after the gap at half speed and about 0.55 s at full speed, with the default
  parameter. That looked wrong.
- **Found by:** converting the delays to PX4 seconds (wall delay times real-time factor): 0.55 s at both speeds. Then a clean
  test with `COM_OF_LOSS_T` set explicitly to 0.5, 1.0 and 2.0 s gave 0.57, 1.05 and 2.0 s. The parameter, not a constant,
  sets the delay, and it counts in PX4 time.
- **Why the earlier numbers were off:** the PX4 SITL start script rewrites timeout parameters whenever
  `PX4_SIM_SPEED_FACTOR` is set, and the simulator keeps parameters between runs, so earlier runs inherited a leftover value.
- **Fix:** the runner now sets `COM_OF_LOSS_T 1.0` explicitly and deletes the saved parameters before every run.
- **Honesty:** the two big sweeps ran before this fix. I kept them and labelled them (`FINDINGS.md` section 3) instead of
  pretending they were clean.

### 5.6b My own instrument was wrong in three ways
- A nominal 50 Hz stream actually ran at 39 Hz, because my scheduler waited for the next 10 ms tick. Fix: schedule from the
  previous due time, not from "now". The achieved rate is now measured from the log.
- The injected gap sat in the middle of a constant-velocity leg, so tracking error could not show any effect. Fix: place it
  across a command change.
- One run per condition was too few near the boundary. Fix: repeats, and a finer grid around 1.0 s.

### 5.7 A bug found by drawing the controller
- **Saw:** while building the on-screen gamepad, the right stick pushed right made the drone move left.
- **Cause:** on a real gamepad the ROS `joy` driver reports right as negative (left and up are positive). My node handled
  that for yaw but not for sideways motion. My first scripted test missed it because the script and the node shared the
  same wrong assumption.
- **Fix:** the mapping moved into a tested pure function, with tests that encode the real-pad convention.
- **Lesson:** a test written with the same wrong assumption as the code proves nothing. Visualising the input found what
  the test could not.

### 5.8 Making real Gazebo footage on a laptop with no usable GPU path
- **Saw:** the recorded window showed only 4 distinct frames per second, and the simulation slowed to 0.90x.
- **Cause:** WSL2 rendered with the CPU (llvmpipe), competing with the physics for the same cores.
- **Fix:** `GALLIUM_DRIVER=d3d12` makes WSL render on the AMD GPU: about 19 distinct frames per second and 0.999x real time
  with the same scene and shadows.
- **Related dead ends I tried and dropped:** Gazebo's built-in video recorder (its service never registers in this setup)
  and capturing the window by title (returns black frames for WSLg windows).

### 5.9 The drone kept disappearing behind things (and a privacy slip)
- **Saw:** poles beside the flight path hid the drone at the start of every act, and a block hid it during a turn.
- **Cause:** a follow camera sits a few metres from the drone, so anything standing near the flight path can pass between them.
- **Fix:** flat markers on the floor instead of posts, tall blocks moved to 30 to 46 m, and a scripted camera operator
  (`camera_director.py`) that sets the camera on a timed plan with smooth motion. I now check the first seconds of every
  recording, not only the middle.
- **A privacy incident, handled:** one screen recording captured my desktop instead of Gazebo because another window took
  focus. I deleted it at once, confirmed it was never committed, and changed the process: minimise other windows first,
  review each recording frame by frame, and keep recordings out of git by rule.

### 5.10 Syncing video to logs without a clapperboard
- Each recording is synced to its flight log automatically. For fixed-camera takes the on-screen motion is correlated with
  the logged speed. For takes with a moving camera, the camera director produces a scripted jump (a marker) at known log
  times, and the largest scene change in the footage gives the offset. The offsets came out at 5.6 to 8.1 s and the
  captions, counters and plots line up with the drone.

### 5.11 Keeping my own machine safe
- A `pip install --user` for PX4's Python tools put newer NumPy and pandas into my WSL user folder, which can break ROS and
  system packages. I noticed, moved that folder aside (not deleted), confirmed the system packages import again, and changed
  the build script to use a virtual environment.
- I also reverted a WSL memory setting I had raised, after Windows killed background jobs for low memory, and made every
  long render write to a temporary file that is renamed only when finished, so a kill can never leave a broken video.
- I checked my thesis files and projects before and after. Nothing was touched.

### 5.12 Making it not look generated
- I rebuilt the video design after feedback that it looked machine-made. What I removed: tiny grey footer text, outlined
  boxes with micro-labels, bracketed HUD tags, glowing tracking rings, a text-block gauge, all-lowercase captions, four equal
  panels. What I kept: a large footage panel, plain plots with real axes, one accent colour used only where something is
  wrong (the tracking ring, the timeout, the failsafe), and sentence-case captions in an open-licence monospace font.

### 5.13 Sound: why my first version was wrong and what the second does
- **First attempt:** a sound track built from the message stream itself: a tick for every fifth setpoint, silence during the
  pause, a riser and a hit at the timeout. Clever on paper. Listening to it I heard clicks and strange tones and no drone,
  so I removed all of it.
- **Second attempt:** a physical model driven by the flight log. The thrust the vehicle needs comes from the logged
  acceleration (the size of g plus a), motor speed from thrust, and the blade-pass pitch is 2 x rpm / 60 for two-blade props.
  Four motors are slightly detuned from each other with a slow wobble that grows with control activity. Forward or sideways
  acceleration speeds up one pair of motors and slows the other (which is how a quad tilts), and yaw biases one diagonal pair.
  Broadband blade and air noise grows with thrust, level and brightness follow the camera distance, and the motors are
  silent when disarmed. Nothing is in the track that a real drone would not make, so it stays in sync with the controls
  because it is computed from them.
- **Check:** a spectrogram shows a line at 181 Hz in hover (180 Hz expected for 5400 rpm) with harmonics above it, pitch bends
  when the drone climbs and when the sticks move, silence before arming and after landing, and no clicks. The level is about
  minus 22 dB RMS in flight.
- **What I cannot claim:** I verified the structure, the pitch and the sync with measurements, not realism by ear. That is why
  there is also a silent version of the film.
## 6. What I learned

1. **Offboard control is a contract about silence, not about speed.** The rule that matters is the longest gap against the
   timeout, and what a good stream looks like depends on time domains.
2. **Time is the hard part of robotics software.** Wall clock, monotonic clock, PX4 time, simulation time and the time the
   middleware reports are five different things, and several of my worst bugs were two of them being mixed up.
3. **Measure the instrument before the system.** My stream rate, my gap placement and even my real-time factor were wrong at
   first. Each was found by checking the measuring tool against an independent reference.
4. **Pin everything you do not study.** Saved parameters and a start script silently changed my experiment. A run is only
   comparable if its conditions are recorded in the log next to its result.
5. **A convenient number can hide the problem.** An accuracy figure hid a 55 ms delay in my earlier project, and a
   real-time factor taken from the wrong clock hid a simulation running at half speed here.
6. **Say what is not proven.** Small samples, unpinned sweeps, scripted joystick input and simulation only are all stated in
   the repository. That is easier to defend than a clean story.
7. **Look at your own output, including the first second.** Several faults (hidden drone, leaked desktop, wrong caption at the
   failsafe) were only visible by checking frames, not by trusting that the pipeline worked.
8. **Make the evidence legible.** The same data became convincing only once the silence counter, the heartbeat and the
   failsafe were on screen at the same time as the footage.

## 7. Limits (read this before presenting)

- Simulation only. One vehicle model (x500), one laptop, 2 to 3 runs per condition. No real hardware.
- The large rate and gap sweeps ran before the timeout parameter was pinned and before the real-time factor was measured per
  run, so their exact thresholds are indicative. The clean evidence is the timeout test (0.5, 1.0, 2.0 s) and the final
  demo flights.
- Simulation speed varied between sessions for the same settings, and at quarter speed some failsafes fired after the
  stream had resumed (a second failure mode I did not investigate).
- The joystick was flown with scripted messages shaped like human stick motion, not with a physical gamepad. The keyboard
  variant was not flown.
- The finding, "the real rule is a timeout on the longest silence", is close to what the documentation implies. It is an
  engineering characterisation, not new science. The more interesting parts are the time-domain results and the failure
  modes found on the way.

## 8. Interview cheat sheet

- **What is it?** A ROS 2 node flying a simulated quadcopter through PX4 offboard mode, plus a measured study of how much
  silence in the command stream PX4 tolerates.
- **Why does offboard need a heartbeat?** PX4 must be able to detect that the companion computer died. It treats a silent
  stream as a failure and takes control back after `COM_OF_LOSS_T`.
- **What would you set for a real vehicle?** Stream at least a few times faster than the timeout allows, never block the
  publishing loop, keep the loop on a monotonic clock, and log the longest gap in flight.
- **What was the hardest bug?** The WSL2 clock stepping back 2.5 s about every 32 s. It froze my timers and made my
  message timestamps look stale to PX4, which then dropped offboard mode. Found with a stack dump and a clock comparison.
- **How do you know the real-time factor?** I read PX4's own clock directly against the monotonic clock, because the XRCE
  agent's time sync hides it. It was 0.5 by default here.
- **What does QoS have to do with it?** PX4 topics are best effort and volatile. A default (reliable) ROS 2 endpoint does not
  match and receives nothing.
- **What would you do on a Jetson or on hardware?** Run the same logging and the same gap test with real latency and a real
  link, add priority scheduling or a C++ node, and compare with the simulation numbers.
- **What would you do next?** Re-run the sweeps with the pinned parameter at measured 1.0x speed, compare with ArduPilot's
  guided mode, and investigate the quarter-speed failure.
- **Did you use AI?** Yes, Claude Code as a pair programmer. I can explain every component and every problem above, and the
  logs are in the repository.

## 9. Reproduce and file map

```
README.md                  overview, how to run
FINDINGS.md                results with caveats, problems and fixes
RESEARCH_QUESTION.md       the question, the gaps in the literature, future work
results/                   all logs, summaries and figures (final, run2, run4, run5, teleop, showcase_raw)
results/videos/            PX4_Offboard_Analysis_White.mp4, PX4_Offboard_Cinematic.mp4
experiments/               experiment runner, analysis, scripted gamepad driver`nvideo/                     Gazebo world, camera director, video composers, drone sound`nsetup/                     environment and PX4 build scripts`ntools/                     small clock and environment checks
ros2_ws/src/px4_offboard_study/   the ROS 2 package and its tests
```

Environment: Windows 11, WSL2 Ubuntu 24.04, ROS 2 Jazzy, PX4 SITL (main branch, October 2026), Gazebo Harmonic, Micro XRCE-DDS
Agent, Ryzen 7 7730U with an AMD integrated GPU.
