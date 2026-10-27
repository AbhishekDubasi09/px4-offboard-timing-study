#!/usr/bin/env bash
# Run every sweep in sequence. Results go to ~/study_results. Inside Ubuntu WSL, from a copy of experiments/ in ~/study (see run_plan.sh).
source /opt/ros/jazzy/setup.bash
cd ~/study
for plan in rate gap jitter load rtf timeout_param; do
  python3 run_experiments.py "$plan"
done
echo ALL_SWEEPS_DONE
