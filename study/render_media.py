"""Render a replay video and screenshots from a flight log.

Usage: python render_media.py <run.csv> <out_dir> [--no-video]
Writes px4_offboard_replay.mp4 (1080p, 30 fps) and px4_offboard_shot_*.png into out_dir.
The log is NED; the plots use east (x), north (y), up (z).
"""
import os
import shutil
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import animation

TEAL, RED, GREY, INK = '#1b6a5a', '#c0504d', '#8a8f98', '#20242b'
FPS = 30
PHASE_LABEL = {'warmup': 'streaming setpoints', 'takeoff': 'takeoff', 'hold': 'hold at 5 m', 'turn': 'yaw turn 90 deg',
               'track': 'velocity square', 'settle': 'settle', 'land': 'landing'}


def load(path):
    df = pd.read_csv(path)
    df = df.dropna(subset=['x']).reset_index(drop=True)
    t = np.arange(df.t.iloc[0], df.t.iloc[-1], 1.0 / FPS)

    def col(name):
        return np.interp(t, df.t, df[name])
    d = {'t': t, 'e': col('y'), 'n': col('x'), 'u': -col('z'), 've': col('vy'), 'vn': col('vx'),
         'ce': col('cmd_ve'), 'cn': col('cmd_vn'), 'yaw': col('heading')}
    idx = np.searchsorted(df.t.to_numpy(), t).clip(0, len(df) - 1)
    d['phase'] = df.phase.to_numpy()[idx]
    d['nav'] = df.nav_state.to_numpy()[idx]
    return d


class Scene:
    def __init__(self, d, size=(19.2, 10.8)):
        self.d = d
        self.fig = plt.figure(figsize=size, dpi=100, facecolor='white')
        gs = self.fig.add_gridspec(3, 2, width_ratios=[1.55, 1], height_ratios=[1.25, 0.8, 0.8],
                                   left=0.03, right=0.975, top=0.90, bottom=0.07, wspace=0.12, hspace=0.42)
        self.a3 = self.fig.add_subplot(gs[:, 0], projection='3d')
        self.am = self.fig.add_subplot(gs[0, 1])
        self.aa = self.fig.add_subplot(gs[1, 1])
        self.av = self.fig.add_subplot(gs[2, 1])
        self.title = self.fig.text(0.03, 0.95, '', fontsize=22, fontweight='bold', color=INK, va='center')
        self.sub = self.fig.text(0.03, 0.918, 'PX4 SITL offboard control from a ROS 2 node (headless Gazebo, x500). Replay drawn from the logged flight data.', fontsize=13, color=GREY)
        self._static()

    def _static(self):
        d = self.d
        a = self.a3
        a.set_facecolor('white')
        for pane in (a.xaxis, a.yaxis, a.zaxis):
            pane.pane.set_facecolor((0.96, 0.97, 0.98, 1)); pane.pane.set_edgecolor('white')
        for pane in (a.xaxis, a.yaxis, a.zaxis):
            pane._axinfo['grid']['color'] = (0.82, 0.85, 0.88, 1)
        a.plot(d['e'], d['n'], np.zeros_like(d['u']), color=TEAL, lw=1, alpha=0.25)  # ground track
        self.trail, = a.plot([], [], [], color=TEAL, lw=2.6)
        self.arms = [a.plot([], [], [], color=INK, lw=4, solid_capstyle='round')[0] for _ in range(2)]
        self.rotors, = a.plot([], [], [], 'o', color=RED, ms=15, alpha=0.9)
        self.body, = a.plot([], [], [], 'o', color=INK, ms=11)
        self.shadow, = a.plot([], [], [], 'o', color='#444', ms=14, alpha=0.18)
        self.drop, = a.plot([], [], [], color=GREY, lw=1, ls=':')
        self.nose, = a.plot([], [], [], color=RED, lw=4)
        a.set_xlabel('east (m)'); a.set_ylabel('north (m)'); a.set_zlabel('altitude (m)')
        a.tick_params(labelsize=9)

        m = self.am
        m.plot(d['e'], d['n'], color=TEAL, lw=1, alpha=0.3)
        self.mtrail, = m.plot([], [], color=TEAL, lw=2.4)
        self.mdot, = m.plot([], [], 'o', color=INK, ms=9)
        self.mcmd, = m.plot([], [], color=RED, lw=3)
        m.set_aspect('equal'); m.set_xlabel('east (m)'); m.set_ylabel('north (m)')
        pad = 1.5
        m.set_xlim(d['e'].min() - pad, d['e'].max() + pad); m.set_ylim(d['n'].min() - pad, d['n'].max() + pad)
        m.set_title('Top view (red = commanded velocity)', loc='left', fontsize=12)

        al = self.aa
        al.plot(d['t'], d['u'], color=TEAL, lw=1, alpha=0.3)
        self.aprog, = al.plot([], [], color=TEAL, lw=2.4)
        self.acur, = al.plot([], [], 'o', color=INK, ms=8)
        al.set_ylabel('altitude (m)'); al.set_title('Altitude', loc='left', fontsize=12)

        v = self.av
        v.plot(d['t'], d['cn'], '--', color=GREY, lw=1.2, label='command N')
        v.plot(d['t'], d['ce'], ':', color=GREY, lw=1.6, label='command E')
        v.plot(d['t'], d['vn'], color=TEAL, lw=1.6, label='measured N')
        v.plot(d['t'], d['ve'], color=RED, lw=1.6, label='measured E')
        self.vcur = v.axvline(0, color=INK, lw=1.2)
        v.set_xlabel('time (s)'); v.set_ylabel('velocity (m/s)')
        v.legend(ncol=4, fontsize=9, loc='upper left', frameon=False)
        v.set_title('Velocity: command vs measured', loc='left', fontsize=12)
        for ax in (m, al, v):
            ax.spines[['top', 'right']].set_visible(False)
        self.aa.set_xlim(d['t'][0], d['t'][-1]); self.av.set_xlim(d['t'][0], d['t'][-1])
        self.av.set_ylim(-1.5, 1.8)

    def frame(self, i, elev=24, azim=None, window=5.2):
        d = self.d
        i = int(np.clip(i, 0, len(d['t']) - 1))
        e, n, u, yaw = d['e'][i], d['n'][i], d['u'][i], d['yaw'][i]
        a = self.a3
        self.trail.set_data_3d(d['e'][:i + 1], d['n'][:i + 1], d['u'][:i + 1])
        fwd = np.array([np.sin(yaw), np.cos(yaw)])
        r = 0.95
        pts = []
        for arm, ang in zip(self.arms, (np.pi / 4, -np.pi / 4)):
            c, s = np.cos(ang), np.sin(ang)
            dv = np.array([fwd[0] * c - fwd[1] * s, fwd[0] * s + fwd[1] * c]) * r
            arm.set_data_3d([e - dv[0], e + dv[0]], [n - dv[1], n + dv[1]], [u, u])
            pts += [(e - dv[0], n - dv[1]), (e + dv[0], n + dv[1])]
        self.rotors.set_data_3d([p[0] for p in pts], [p[1] for p in pts], [u] * 4)
        self.body.set_data_3d([e], [n], [u])
        self.nose.set_data_3d([e, e + fwd[0] * r * 1.3], [n, n + fwd[1] * r * 1.3], [u, u])
        self.shadow.set_data_3d([e], [n], [0.0])
        self.drop.set_data_3d([e, e], [n, n], [0, u])
        w = window
        a.set_xlim3d(e - w, e + w); a.set_ylim3d(n - w, n + w); a.set_zlim3d(0, max(2 * w * 0.75, u + 1.5))
        a.set_box_aspect((1, 1, 0.75))
        a.view_init(elev=elev, azim=-58 + 20 * (i / len(d['t'])) if azim is None else azim)

        self.mtrail.set_data(d['e'][:i + 1], d['n'][:i + 1])
        self.mdot.set_data([e], [n])
        self.mcmd.set_data([e, e + d['ce'][i] * 2.5], [n, n + d['cn'][i] * 2.5])
        self.aprog.set_data(d['t'][:i + 1], d['u'][:i + 1])
        self.acur.set_data([d['t'][i]], [u])
        self.vcur.set_xdata([d['t'][i], d['t'][i]])
        mode = 'OFFBOARD' if d['nav'][i] == 14 else ('landing' if d['nav'][i] == 18 else 'PX4 mode ' + str(int(d['nav'][i])))
        self.title.set_text(f"t = {d['t'][i]:5.1f} s   {PHASE_LABEL.get(d['phase'][i], d['phase'][i])}   |   {mode}")

    def save(self, i, path, **kw):
        self.frame(i, **kw)
        self.fig.savefig(path, dpi=100)


def first(d, phase, frac=0.5):
    idx = np.where(d['phase'] == phase)[0]
    return int(idx[int(len(idx) * frac)]) if len(idx) else 0


def main():
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)
    d = load(src)
    sc = Scene(d)
    shots = [('01_takeoff', first(d, 'takeoff', 0.8), dict(azim=-55, elev=22)),
             ('02_hold_and_turn', first(d, 'turn', 0.7), dict(azim=-35, elev=28)),
             ('03_velocity_square_east_leg', first(d, 'track', 0.45), dict(azim=-70, elev=20)),
             ('04_velocity_square_return_leg', first(d, 'track', 0.85), dict(azim=-20, elev=30)),
             ('05_landing', first(d, 'land', 0.55), dict(azim=-50, elev=18))]
    for name, i, kw in shots:
        sc.save(i, os.path.join(out, f'px4_offboard_shot_{name}.png'), **kw)
        print('shot', name)
    if '--no-video' in sys.argv:
        return
    ff = shutil.which('ffmpeg') or r'C:\Users\abhib\AppData\Local\Microsoft\WinGet\Packages\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-9.0.2-full_build\bin\ffmpeg.exe'
    matplotlib.rcParams['animation.ffmpeg_path'] = ff
    w = animation.FFMpegWriter(fps=FPS, codec='libx264', bitrate=9000,
                               extra_args=['-pix_fmt', 'yuv420p', '-preset', 'medium'])
    path = os.path.join(out, 'px4_offboard_replay.mp4')
    with w.saving(sc.fig, path, dpi=100):
        for i in range(len(d['t'])):
            sc.frame(i)
            w.grab_frame()
            if i % 150 == 0:
                print('frame', i, '/', len(d['t']), flush=True)
    print('video', path)


if __name__ == '__main__':
    main()
