#!/usr/bin/env bash
# Copy the showcase world, its texture and the minimal GUI config to where PX4 and Gazebo look for them.
S=/mnt/c/Users/abhib/px4_offboard_study/video
mkdir -p ~/gz_assets && cp $S/assets/checker.png ~/gz_assets/checker.png
cp $S/assets/showcase.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/showcase.sdf
cp $S/gui_minimal.config ~/gz_gui_minimal.config
ls -la ~/gz_assets ~/PX4-Autopilot/Tools/simulation/gz/worlds/showcase.sdf | cut -c1-100
