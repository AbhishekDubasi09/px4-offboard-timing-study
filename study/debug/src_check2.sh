#!/usr/bin/env bash
cd ~/PX4-Autopilot/src/modules
ls flight_mode_manager/tasks | head -20
echo ---; grep -rln "offboard" flight_mode_manager | head
echo ---; grep -rn "hrt_elapsed_time\|_ms\|timeout" flight_mode_manager/tasks/Offboard 2>/dev/null | head
