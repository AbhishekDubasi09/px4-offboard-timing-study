#!/usr/bin/env bash
# Read-only check: is the thesis material in this WSL home untouched, and what did the PX4 setup change in Python?
echo "--- thesis folder"; ls -la ~/thesis 2>&1 | head -12
echo "--- files under ~/thesis, ~/notebooks, ~/ros2_ws, ~/sim_ws modified since 26 Oct (should be none, or only your own work):"
find ~/thesis ~/notebooks ~/ros2_ws/src ~/sim_ws/src -type f -newermt '2026-10-26' -not -path '*/.git/*' 2>/dev/null | head -15
echo "--- user-site Python packages touched since 26 Oct (my pip install of PX4 requirements):"
find ~/.local/lib/python3.12/site-packages -maxdepth 1 -newermt '2026-10-26' 2>/dev/null | sed 's#.*/##' | sort | head -60
echo "--- numpy / pandas now:"; python3 -c "import numpy; print('numpy', numpy.__version__)" 2>&1 | tail -1; python3 -c "import pandas; print('pandas', pandas.__version__)" 2>&1 | tail -1
