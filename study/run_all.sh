#!/usr/bin/env bash
# Run every sweep in sequence (about 60 minutes). Results go to ~/study_results.
source /opt/ros/jazzy/setup.bash
cd ~/study
for plan in rate gap jitter load speed; do
  python3 run_experiments.py "$plan"
done
echo ALL_SWEEPS_DONE
