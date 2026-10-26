"""Summarise runs: per-run tracking error and offboard outcome. Usage: python analyze.py <results_dir>"""
import glob
import json
import os
import sys

import numpy as np
import pandas as pd

d = sys.argv[1]
rows = []
for jf in sorted(glob.glob(os.path.join(d, '*.json'))):
    name = os.path.basename(jf)[:-5]
    r = json.load(open(jf))
    df = pd.read_csv(os.path.join(d, name + '.csv'))
    tr = df[df.phase == 'track']
    err = np.hypot(tr.vx - tr.cmd_vn, tr.vy - tr.cmd_ve) if len(tr) else pd.Series(dtype=float)
    sent = df[df.sent == 1].t.diff().dropna()
    r.update(run=name, rms_vel_err=float(np.sqrt((err ** 2).mean())) if len(err) else None,
             max_gap_s=float(sent.max()) if len(sent) else None,
             mean_rate_hz=float(1 / sent.mean()) if len(sent) else None)
    rows.append(r)
out = pd.DataFrame(rows)
out.to_csv(os.path.join(d, 'summary.csv'), index=False)
print(out[['run', 'offboard_lost', 'exit_time_s', 'rms_vel_err', 'max_gap_s', 'mean_rate_hz']].to_string())
