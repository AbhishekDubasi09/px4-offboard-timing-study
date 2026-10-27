#!/usr/bin/env bash
cd ~/PX4-Autopilot
grep -rn "COM_OF_LOSS_T" ROMFS Tools/simulation 2>/dev/null | grep -v "_LONGER" | head -5
