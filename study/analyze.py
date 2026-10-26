"""Summarise runs and draw figures. Usage: python analyze.py <results_dir> [<figures_dir>]

Per run: offboard outcome, failsafe latency, velocity-tracking RMS error during the track phase,
achieved stream rate and longest gap between setpoints (airborne phases only), and the measured
real-time factor (PX4 seconds per wall second, from PX4 timestamps against the node's steady clock).
"""
import glob
import json
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

GREY = '#8a8f98'
AIR = ('hold', 'turn', 'track', 'settle')


def run_stats(path_csv, meta):
    df = pd.read_csv(path_csv)
    air = df[df.phase.isin(AIR)]
    out = {}
    # Real-time factor comes from the runner (PX4's own clock vs the monotonic clock). The XRCE agent's
    # timesync makes the timestamps in these CSVs follow wall time, so they cannot be used for it.
    if meta.get('rtf_px4_mono') is not None:
        out['rtf'] = meta['rtf_px4_mono']
    sent = air[air.sent == 1].t.to_numpy()
    if len(sent) > 2:
        out['achieved_rate_hz'] = (len(sent) - 1) / (sent[-1] - sent[0]) if meta.get('gap_len', 0) == 0 else np.nan
        out['max_gap_s'] = float(np.diff(sent).max())
    tr = df[df.phase == 'track']
    if len(tr):
        err = np.hypot(tr.vx - tr.cmd_vn, tr.vy - tr.cmd_ve)
        out['rms_vel_err'] = float(np.sqrt((err ** 2).mean()))
    if meta.get('exit_time_s') is not None and meta.get('gap_onset_s') is not None:
        out['failsafe_latency_s'] = meta['exit_time_s'] - meta['gap_onset_s']
    if meta.get('gap_len', 0) > 0:  # the node stops counting once offboard is lost, so use the injected gap
        out['max_gap_s'] = max(out.get('max_gap_s', 0.0), meta['gap_len'])
    if 'max_gap_s' in out and 'rtf' in out:
        out['max_gap_px4_s'] = out['max_gap_s'] * out['rtf']
    return out


def load(d):
    rows = []
    for jf in sorted(glob.glob(os.path.join(d, '*.json'))):
        name = os.path.basename(jf)[:-5]
        meta = json.load(open(jf))
        row = {'run': name, 'cond': re.sub(r'_s\d+$', '', name), **meta}
        row.update(run_stats(os.path.join(d, name + '.csv'), meta))
        rows.append(row)
    return pd.DataFrame(rows)


def figures(df, d, fd):
    os.makedirs(fd, exist_ok=True)
    GREY_ = None
    if 'failsafe_latency_s' not in df:
        df['failsafe_latency_s'] = np.nan
    plt.rcParams.update({'font.size': 11, 'axes.spines.top': False, 'axes.spines.right': False})

    r = df[df.run.str.startswith('rate_')].copy()
    if len(r):
        g = r.groupby('rate_hz').agg(err=('rms_vel_err', 'mean'), lost=('offboard_lost', 'mean')).reset_index()
        fig, ax = plt.subplots(figsize=(7, 4.2))
        ax.plot(g.rate_hz, g.err, 'o-', color='#1b6a5a')
        ax.axvline(2, ls='--', color='grey')
        ax.text(2.1, ax.get_ylim()[1] * 0.97, 'documented 2 Hz minimum', va='top', color='grey')
        for _, row in g[g.lost > 0].iterrows():
            ax.plot(row.rate_hz, ax.get_ylim()[0], 'rx', ms=10, mew=2)
        ax.set_xscale('log'); ax.set_xlabel('steady setpoint rate (Hz)'); ax.set_ylabel('RMS velocity error (m/s)')
        ax.set_title('Tracking error vs steady stream rate (x = offboard lost)')
        fig.tight_layout(); fig.savefig(os.path.join(fd, 'fig1_rate_vs_error.png'), dpi=200); plt.close(fig)

    gp = df[df.run.str.startswith('gap_')].copy()
    if len(gp):
        g = gp.groupby('gap_len').agg(lost=('offboard_lost', 'mean'), n=('run', 'count'),
                                      lat=('failsafe_latency_s', 'mean')).reset_index()
        fig, ax = plt.subplots(figsize=(7, 4.2))
        ax.bar(g.gap_len.astype(str), g.lost * 100, color='#c0504d')
        for i, row in g.iterrows():
            ax.text(i, row.lost * 100 + 2, f'n={int(row.n)}', ha='center')
        ax.set_xlabel('single gap in a 50 Hz stream (s)'); ax.set_ylabel('runs that lost offboard (%)')
        ax.set_title('Offboard loss vs gap length (default COM_OF_LOSS_T = 1.0 s)')
        fig.tight_layout(); fig.savefig(os.path.join(fd, 'fig2_gap_vs_loss.png'), dpi=200); plt.close(fig)

    rt = df[df.run.str.startswith('rtf')].copy()
    if len(rt) and 'rtf' in rt:
        rt['rtf_r'] = rt.rtf.round(2)
        fig, (ax, bx) = plt.subplots(1, 2, figsize=(12, 4.4))
        cols = {0.25: '#c0504d', 0.5: '#e0a030', 1.0: '#1b6a5a'}
        for r_, sub in rt.groupby('rtf_r'):
            g = sub.groupby('gap_len').offboard_lost.mean().reset_index()
            ax.plot(g.gap_len, g.offboard_lost * 100, 'o-', color=cols.get(r_, GREY), label=f'real-time factor {r_:g}')
        ax.set_xlabel('gap length in wall-clock seconds'); ax.set_ylabel('runs that lost offboard (%)')
        ax.legend(); ax.set_title('Loss vs wall-clock gap at three simulation speeds')
        # Latency in PX4 seconds = wall latency x real-time factor. Flat across speeds means the timeout runs
        # in PX4 time. Only runs where the failsafe came while the gap was still open are used: at quarter
        # speed some failsafes fired after the stream had resumed, which is a different failure.
        rt['lat_px4'] = rt.failsafe_latency_s * rt.rtf
        ok = rt[rt.offboard_lost & (rt.failsafe_latency_s <= rt.gap_len + 0.15)]
        lat = ok.groupby('rtf_r').lat_px4.agg(['mean', 'min', 'max', 'count']).reset_index()
        bx.bar(lat.rtf_r.astype(str), lat['mean'], color=[cols.get(v, GREY) for v in lat.rtf_r])
        bx.errorbar(np.arange(len(lat)), lat['mean'], yerr=[lat['mean'] - lat['min'], lat['max'] - lat['mean']],
                    fmt='none', ecolor='black', capsize=5)
        for i, row in lat.iterrows():
            bx.text(i, row['max'] + 0.03, f"n={int(row['count'])}", ha='center')
        bx.set_ylim(0, 1.1)
        bx.set_xlabel('real-time factor'); bx.set_ylabel('PX4 seconds from gap start to leaving offboard')
        bx.set_title('Failsafe latency in PX4 time')
        late = rt[rt.offboard_lost & (rt.failsafe_latency_s > rt.gap_len + 0.15)]
        fig.text(0.99, 0.005, f'Flat bars = the timeout runs in PX4 time. Left out: {len(late)} failsafes that fired after the '
                 'stream had resumed (all at 0.25x).', ha='right', fontsize=9, color=GREY)
        fig.tight_layout(); fig.savefig(os.path.join(fd, 'fig3_realtime_factor.png'), dpi=200); plt.close(fig)

    ex = os.path.join(d, 'demo.csv')
    if os.path.exists(ex):
        t = pd.read_csv(ex)
        fig, ax = plt.subplots(3, 1, figsize=(8, 6.5), sharex=True)
        ax[0].plot(t.t, -t.z, color='#1b6a5a'); ax[0].set_ylabel('altitude (m)')
        ax[1].plot(t.t, t.cmd_vn, '--', color='grey', label='command N'); ax[1].plot(t.t, t.vx, color='#1b6a5a', label='measured N')
        ax[1].plot(t.t, t.cmd_ve, ':', color='grey', label='command E'); ax[1].plot(t.t, t.vy, color='#c0504d', label='measured E')
        ax[1].set_ylabel('velocity (m/s)'); ax[1].legend(ncol=4, fontsize=8, loc='upper left')
        ax[2].step(t.t, t.nav_state, color='black'); ax[2].set_ylabel('nav_state\n(14 = offboard)'); ax[2].set_xlabel('time (s)')
        fig.suptitle('Offboard demo flight: takeoff, turn, velocity square, land'); fig.tight_layout()
        fig.savefig(os.path.join(fd, 'fig0_demo_flight.png'), dpi=200); plt.close(fig)


if __name__ == '__main__':
    d = sys.argv[1]
    fd = sys.argv[2] if len(sys.argv) > 2 else os.path.join(d, 'figures')
    df = load(d)
    df.to_csv(os.path.join(d, 'summary.csv'), index=False)
    cols = ['run', 'offboard_lost', 'failsafe_latency_s', 'rms_vel_err', 'achieved_rate_hz', 'max_gap_s', 'rtf']
    print(df[[c for c in cols if c in df]].round(3).to_string())
    figures(df, d, fd)
