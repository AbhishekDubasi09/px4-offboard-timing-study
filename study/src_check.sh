#!/usr/bin/env bash
cd ~/PX4-Autopilot
echo "== COM_OF_LOSS_T definition"; grep -rn -B2 -A12 "COM_OF_LOSS_T" src/modules/commander/commander_params.c 2>/dev/null | grep -E "COM_OF_LOSS_T|@(min|max|unit|group)|PARAM_DEFINE" | head
echo "== uses"; grep -rn "of_loss_t\|OF_LOSS" src --include=*.cpp --include=*.hpp | head -8
echo "== offboard availability check"; grep -rn "offboard_control_mode" src/modules/commander/*.cpp src/modules/commander/*/*.cpp 2>/dev/null | grep -iE "timestamp|elapsed|timeout|available" | head -8
