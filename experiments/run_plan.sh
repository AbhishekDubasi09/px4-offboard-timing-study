#!/usr/bin/env bash
# Usage: run_plan.sh <plan> <out_dir_name>   (inside Ubuntu WSL; copies the repo's experiments/ first)
source /opt/ros/jazzy/setup.bash
rm -rf ~/study && cp -r /mnt/c/Users/abhib/px4_offboard_study/experiments ~/study
cd ~/study && STUDY_OUT=$HOME/$2 python3 run_experiments.py "$1"
