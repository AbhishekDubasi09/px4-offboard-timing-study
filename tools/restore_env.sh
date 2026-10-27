#!/usr/bin/env bash
# Undo what the PX4 setup added to the user Python site in this WSL distro (moved aside, not deleted).
S=~/.local/lib/python3.12
[ -d $S/site-packages ] && mv $S/site-packages $S/site-packages.px4_backup && mkdir -p $S/site-packages
echo "--- imports now come from the system packages:"
python3 -c "import numpy; print('numpy', numpy.__version__, numpy.__file__)" 2>&1 | tail -1
python3 -c "import pandas; print('pandas', pandas.__version__)" 2>&1 | tail -1
python3 -c "import matplotlib; print('matplotlib', matplotlib.__version__)" 2>&1 | tail -1
source /opt/ros/jazzy/setup.bash 2>/dev/null; python3 -c "import rclpy; print('rclpy OK')" 2>&1 | tail -1
python3 -c "import cv_bridge; print('cv_bridge OK')" 2>&1 | tail -1
echo "--- thesis venv untouched:"; ls ~/thesis/.venv | head -5
