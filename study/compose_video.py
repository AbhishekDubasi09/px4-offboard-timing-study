"""Compose the showcase video: real Gazebo footage, synced to the flight logs, with controller, captions and plots.

Usage: python compose_video.py <out.mp4>              full video
       python compose_video.py --shots <dir>          save one test frame per act and the cards, then exit
Inputs: results/showcase_raw/act_<key>.mp4 + act_<key>.csv for A_teleop, B_gap0.6, C_gap1.5.
"""
import math
import os
import subprocess
import sys

import matplotlib
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'ros2_ws', 'src', 'px4_offboard_study'))
from px4_offboard_study.mathutil import body_to_ned, shape  # noqa: E402

FF = r'C:\Users\abhib\AppData\Local\Microsoft\WinGet\Packages\Gyan.FFmpeg_Microsoft.Winget.Source_8wekyb3d8bbwe\ffmpeg-9.0.2-full_build\bin\ffmpeg.exe'
RAW = os.path.join(HERE, '..', 'results', 'showcase_raw')
W, H, FPS = 1920, 1080, 30
VX, VY, VW, VH = 24, 88, 1240, 696           # Gazebo panel
CX, CY, CW, CH = 1288, 88, 608, 390          # controller / experiment card
KX, KY, KW, KH = 1288, 492, 608, 292         # caption box
PLOTS = [(24, 800, 608, 256), (656, 800, 608, 256), (1288, 800, 608, 256)]
AX = (0.115, 0.17, 0.865, 0.62)              # plot axes position (fraction of figure)

BG, PANEL, BORDER = (13, 21, 32), (20, 32, 51), (38, 54, 77)
TEXT, MUTED = (232, 240, 248), (138, 160, 184)
TEAL, ORANGE, RED, AMBER, BLUE = (25, 211, 162), (255, 112, 45), (255, 59, 92), (255, 194, 51), (76, 201, 240)
TIMEOUT_S = 1.0

_fonts = {}


def font(size, bold=False):
    key = (size, bold)
    if key not in _fonts:
        name = 'segoeuib.ttf' if bold else 'segoeui.ttf'
        _fonts[key] = ImageFont.truetype(os.path.join(r'C:\Windows\Fonts', name), size)
    return _fonts[key]


def wrap(draw, text, fnt, maxw):
    lines, line = [], ''
    for word in text.split():
        trial = (line + ' ' + word).strip()
        if draw.textlength(trial, font=fnt) <= maxw:
            line = trial
        else:
            lines.append(line)
            line = word
    lines.append(line)
    return lines


def hexc(c):
    return '#%02x%02x%02x' % c


# ----------------------------------------------------------------------------------------- data


class Act:
    def __init__(self, key, kind, title, goal, gap=0.0):
        self.key, self.kind, self.title, self.goal, self.gap = key, kind, title, goal, gap
        df = pd.read_csv(os.path.join(RAW, f'act_{key}.csv'))
        for c in ['x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'nav_state']:
            df[c] = pd.to_numeric(df[c], errors='coerce')
        self.df = df = df.ffill().bfill().reset_index(drop=True)
        self.t = df.t.to_numpy()
        self.alt = -df.z.to_numpy()
        self.sph = np.hypot(df.vx, df.vy).to_numpy()
        self.nav = df.nav_state.to_numpy()
        self.armed = df.armed.to_numpy()
        self.heading = df.heading.to_numpy()
        self.speed3 = np.hypot(self.sph, df.vz.to_numpy())
        if kind == 'teleop':
            n = len(df)
            fwd = [shape(v, 0.1, 2.0) for v in df.ax_fwd]
            right = [shape(-v, 0.1, 2.0) for v in df.ax_right]
            self.ned = np.array([body_to_ned(f, r, 0.0, h) for f, r, h in zip(fwd, right, self.heading)])
            self.ned[:, 2] = -np.array([shape(v, 0.1, 1.0) for v in df.ax_up])
            self.cmd_h = np.hypot(self.ned[:, 0], self.ned[:, 1])
            self.since = np.minimum(self.t - np.floor(self.t / 0.05) * 0.05, 0.05)  # nominal 20 Hz stream
            self.sent_t = np.arange(self.t[0], self.t[-1], 0.05)
            self.stage = df.label.to_numpy()
            self.yaw_rate = np.array([shape(-v, 0.1, 0.8) for v in df.ax_yaw])
        else:
            self.cmd_h = np.hypot(df.cmd_vn, df.cmd_ve).to_numpy()
            sent = df.sent.to_numpy()
            last, since = self.t[0], np.zeros(len(df))
            for i, (tt, s) in enumerate(zip(self.t, sent)):
                if s:
                    last = tt
                since[i] = tt - last
            self.since, self.sent_t = since, self.t[sent == 1]
            self.stage = df.phase.to_numpy()
            self.track_t0 = self.t[np.argmax(self.stage == 'track')]
            after = np.nonzero((self.t > self.track_t0) & (since > 0.12))[0]
            self.gap_on = self.t[after[0]] if len(after) else None
            res = np.nonzero((self.t > (self.gap_on or 0)) & (sent == 1))[0] if self.gap_on else []
            self.gap_off = self.t[res[0]] if len(res) else None
        ex = np.nonzero((self.nav[1:] != 14) & (self.nav[:-1] == 14) & (self.t[1:] > 3))[0]
        self.exit_t = self.t[ex[0] + 1] if len(ex) and self.nav[ex[0] + 1] == 5 else None
        armed = np.nonzero(self.armed == 1)[0]
        self.t_arm = self.t[armed[0]]
        self.t_end = min(self.t[armed[-1]] + 3.0, self.t[-1])
        self.off = None

    def segments(self):
        a = self.t_arm - 1.0
        if self.kind == 'teleop':
            land = self.t[np.argmax(self.stage == 'land')]
            return [(a, 13.0, 2.0), (13.0, 45.5, 1.0), (45.5, 51.5, 2.0), (51.5, land + 4.0, 1.0), (land + 4.0, self.t_end, 2.0)]
        stop = (self.exit_t + 9.0) if self.exit_t else (self.gap_off + 9.0)
        slow_end = (self.exit_t + 2.2) if self.exit_t else (self.gap_off + 1.2)
        s0 = self.gap_on - 0.8   # slow motion across the pause so the countdown is readable
        return [(a, self.track_t0 - 4.0, 2.0), (self.track_t0 - 4.0, s0, 1.0), (s0, slow_end, 0.4), (slow_end, stop, 1.0), (stop, self.t_end, 2.0)]

    def times(self):
        out = []
        for a, b, sp in self.segments():
            t = a
            while t < b:
                out.append((t, sp))
                t += sp / FPS
        return out

    def idx(self, t):
        return int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))

    def detect_offset(self):
        path = os.path.join(RAW, f'act_{self.key}.mp4')
        raw = subprocess.run([FF, '-v', 'error', '-t', '55', '-i', path, '-vf', 'fps=10,scale=96:54,format=gray',
                              '-f', 'rawvideo', '-'], capture_output=True).stdout
        fr = np.frombuffer(raw, np.uint8).reshape(-1, 54, 96).astype(np.float32)
        m = np.abs(np.diff(fr, axis=0)).mean(axis=(1, 2))
        tm = (np.arange(len(m)) + 1) / 10.0
        best, off_best = -2, 0
        for off in np.arange(0, 12, 0.1):
            ts = tm - off
            ok = ts >= 0
            s = np.interp(ts[ok], self.t, self.speed3)
            r = np.corrcoef(m[ok], s)[0, 1]
            if r > best:
                best, off_best = r, off
        self.off = round(float(off_best), 1)
        print(f'{self.key}: video offset {self.off} s (corr {best:.2f})')


# ----------------------------------------------------------------------------------------- captions


def caption(act, t):
    i = act.idx(t)
    st = act.stage[i]
    if act.kind == 'teleop':
        if t < act.t_arm + 0.3:
            return ('Stream first, then switch mode', 'The ROS 2 node publishes setpoints at 20 Hz BEFORE it asks PX4 for offboard mode. Without that stream PX4 refuses the switch.', 'info')
        if st == 'idle' and t < 13.5:
            return ('Arm, offboard, climb to 5 m', 'PX4 now follows the ROS 2 node instead of a pilot: a position setpoint 5 m up, streamed continuously.', 'info')
        table = {
            'idle': ('Sticks centred: hold position', 'Zero velocity setpoint. The 20 Hz stream keeps flowing, so PX4 stays in offboard mode.', 'info'),
            'forward': ('Right stick forward', 'Stick value goes through a deadzone and a 2 m/s limit, then body frame to NED using the drone\'s heading (it faces east).', 'info'),
            'right': ('Right stick sideways', 'Same pipeline: the right-hand side of an east-facing drone is south. This sign was inverted in my first version; the overlay exposed it.', 'warn'),
            'climb': ('Left stick up: climb', 'Vertical speed is limited to 1 m/s and sent as a NED down-velocity (negative = up).', 'info'),
            'descend': ('Left stick down: descend', 'Same limit in the other direction. Altitude plot (bottom middle) follows the stick.', 'info'),
            'carve': ('Forward + yaw sticks: carving turn', 'Both sticks at once. The body-to-NED conversion keeps working while the heading changes, so the path bends smoothly.', 'info'),
            'land': ('A button: land', 'Land command sent. PX4 lands and disarms. The stream is no longer needed.', 'ok'),
        }
        return table.get(st, table['idle'])
    if st == 'warmup':
        return ('Stream first, then switch mode', 'The script publishes setpoints at 20 Hz BEFORE it asks PX4 for offboard mode. Without a stream PX4 refuses the switch.', 'info')
    if st == 'takeoff':
        return ('Arm, offboard, climb to 5 m', 'A position setpoint 5 m up, streamed continuously. PX4 follows this ROS 2 node.', 'info')
    if st == 'hold':
        return ('Hold position', 'The stream keeps flowing even though nothing changes: the heartbeat must never stop.', 'info')
    if st == 'turn':
        return ('Yaw turn, 90 degrees', 'Still streaming at 20 Hz: one heartbeat and one setpoint every 50 ms.', 'info')
    gap_txt = f'{act.gap:g} s'
    if st == 'track':
        if act.gap_on and t < act.gap_on:
            return ('Velocity square at 20 Hz', 'Longest silence so far: 0.05 s. PX4 only complains after 1.0 s (COM_OF_LOSS_T): a 20x margin.', 'info')
        if act.gap_off and t < act.gap_off and not (act.exit_t and t >= act.exit_t):
            return (f'TEST: the stream is paused for {gap_txt}', f'No message arrives. PX4 counts the silence against its {TIMEOUT_S:g} s offboard timeout. Watch the gauge (top right) and the stream plot.', 'warn')
        if act.exit_t and t >= act.exit_t:
            return ('TIMEOUT: PX4 leaves offboard mode', 'After 1.0 s of silence PX4 assumes the companion computer has died and takes control: return-to-launch. This is the failure the heartbeat rule prevents.', 'alert')
        return ('Stream is back: offboard mode kept', f'{gap_txt} of silence is below the {TIMEOUT_S:g} s timeout, so no failsafe. Only the velocity command that changed during the gap was missed.', 'ok')
    if st == 'settle':
        return ('Square finished', 'Back to steady 20 Hz streaming; the drone settles.', 'info')
    if act.exit_t:
        return ('Failsafe landing', 'PX4 is no longer in offboard mode, so my script just commands a landing. The test is over.', 'alert')
    return ('Landing', 'The script sends a land command and stops streaming. In land mode silence is harmless.', 'ok')


PHASE_TAG = {'warmup': 'STREAMING', 'takeoff': 'TAKEOFF', 'hold': 'HOLD', 'turn': 'YAW TURN', 'track': 'VELOCITY SQUARE',
             'settle': 'SETTLE', 'land': 'LANDING', 'idle': 'HOLD', 'forward': 'FORWARD', 'right': 'SIDEWAYS',
             'climb': 'CLIMB', 'descend': 'DESCEND', 'carve': 'CARVING TURN'}


# ----------------------------------------------------------------------------------------- plots


def make_plots(act):
    t0, t1 = act.times()[0][0], act.times()[-1][0]
    m = (act.t >= t0) & (act.t <= t1)
    t = act.t[m]
    out = []
    plt.rcParams.update({'font.size': 9, 'text.color': hexc(TEXT), 'axes.labelcolor': hexc(MUTED),
                         'xtick.color': hexc(MUTED), 'ytick.color': hexc(MUTED)})

    def fig_axes(title, raster=False):
        fig = plt.figure(figsize=(PLOTS[0][2] / 100, PLOTS[0][3] / 100), dpi=100, facecolor=hexc(PANEL))
        if raster:
            ax = fig.add_axes([AX[0], AX[1], AX[2], 0.40])
            rx = fig.add_axes([AX[0], 0.64, AX[2], 0.14], sharex=ax)
            rx.set_facecolor('#0f1a2a')
            rx.set_yticks([])
            rx.tick_params(labelbottom=False)
            for s in rx.spines.values():
                s.set_color(hexc(BORDER))
        else:
            ax, rx = fig.add_axes([AX[0], AX[1], AX[2], AX[3]]), None
        ax.set_facecolor('#0f1a2a')
        for s in ax.spines.values():
            s.set_color(hexc(BORDER))
        ax.grid(color=hexc(BORDER), lw=0.6)
        fig.text(0.02, 0.93, title, fontsize=11, fontweight='bold', color=hexc(TEXT), va='center')
        ax.set_xlim(t0, t1)
        return fig, ax, rx

    def band(ax):
        nav = act.nav[m]
        colors = {14: (*[c / 255 for c in TEAL], 0.16), 5: (*[c / 255 for c in RED], 0.30),
                  18: (*[c / 255 for c in AMBER], 0.22), 4: (0.5, 0.5, 0.5, 0.12)}
        start = 0
        for i in range(1, len(nav) + 1):
            if i == len(nav) or nav[i] != nav[start]:
                ax.axvspan(t[start], t[min(i, len(t) - 1)], color=colors.get(int(nav[start]), (0.4, 0.4, 0.4, 0.1)), lw=0)
                start = i

    # 1: speed command vs measured
    fig, ax, _ = fig_axes('Horizontal speed: command vs measured')
    band(ax)
    ax.plot(t, act.cmd_h[m], color=hexc(ORANGE), lw=1.8, ls='--', label='command (setpoint to PX4)')
    ax.plot(t, act.sph[m], color=hexc(TEAL), lw=1.8, label='measured by PX4')
    ax.set_ylabel('m/s')
    ax.set_xlabel('time (s)')
    ax.legend(loc='upper left', fontsize=8, frameon=False, labelcolor=hexc(TEXT), ncol=2, bbox_to_anchor=(0.0, 1.0))
    out.append(fig)
    # 2: altitude + mode
    fig, ax, _ = fig_axes('Altitude and PX4 flight mode')
    band(ax)
    ax.plot(t, act.alt[m], color=hexc(BLUE), lw=2.0)
    ax.set_ylabel('altitude (m)')
    ax.set_xlabel('time (s)')
    ax.set_ylim(-0.2, max(7.0, act.alt[m].max() + 1.0))
    for k, (lab, col) in enumerate([('OFFBOARD', TEAL), ('FAILSAFE RETURN', RED), ('LAND', AMBER)]):
        ax.text(0.99 - 0.0, 0.93 - 0.11 * k, lab, transform=ax.transAxes, ha='right', fontsize=8, color=hexc(col), fontweight='bold')
    out.append(fig)
    # 3: setpoint stream
    fig, ax, rx = fig_axes('Setpoint stream and the 1.0 s timeout', raster=True)
    band(ax)
    st = act.sent_t[(act.sent_t >= t0) & (act.sent_t <= t1)]
    rx.vlines(st, 0, 1, color=hexc(TEAL), lw=0.6)
    rx.set_ylim(0, 1)
    rx.text(0.005, 0.5, 'each tick = one message', transform=rx.transAxes, fontsize=7, color=hexc(MUTED), va='center')
    ax.plot(t, np.minimum(act.since[m], 1.5), color=hexc(AMBER), lw=1.8)
    ax.axhline(TIMEOUT_S, color=hexc(RED), lw=1.4, ls='--')
    ax.text(t0 + 0.01 * (t1 - t0), TIMEOUT_S + 0.04, 'PX4 timeout 1.0 s', color=hexc(RED), fontsize=8, fontweight='bold')
    ax.set_ylim(0, 1.5)
    ax.set_ylabel('silence (s)')
    ax.set_xlabel('time (s)')
    if act.exit_t:
        ax.axvline(act.exit_t, color=hexc(RED), lw=1.6)
        ax.text(act.exit_t - 0.01 * (t1 - t0), 1.35, 'FAILSAFE', color=hexc(RED), fontsize=8, fontweight='bold', ha='right')
    out.append(fig)
    res = []
    for fig in out:
        fig.canvas.draw()
        full = np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()
        plt.close(fig)
        dim = full.copy()   # dim only the data area; titles and labels stay bright
        c0, c1 = int(AX[0] * PLOTS[0][2]), int((AX[0] + AX[2]) * PLOTS[0][2])
        dim[54:212, c0:c1] = (full[54:212, c0:c1] * 0.35 + np.array((15, 26, 42)) * 0.65).astype(np.uint8)
        res.append((full, dim))
    return res, (t0, t1)


# ----------------------------------------------------------------------------------------- drawing


def rrect(d, box, r, fill=None, outline=None, width=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=width)


def static_background():
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    rrect(d, (VX - 4, VY - 4, VX + VW + 4, VY + VH + 4), 10, fill=PANEL, outline=BORDER, width=2)
    rrect(d, (CX, CY, CX + CW, CY + CH), 10, fill=PANEL, outline=BORDER, width=2)
    rrect(d, (KX, KY, KX + KW, KY + KH), 10, fill=PANEL, outline=BORDER, width=2)
    d.text((KX + 18, KY + 12), 'WHAT IS HAPPENING', font=font(14, True), fill=MUTED)
    d.text((24, H - 20), 'Real Gazebo footage (GPU rendered), PX4 SITL, simulation only. Logs, code and caveats: FINDINGS.md', font=font(13), fill=MUTED)
    return np.array(img)


def draw_header(d, act, t, speed, nav):
    d.text((24, 14), 'PX4 OFFBOARD CONTROL', font=font(15, True), fill=TEAL)
    d.text((24, 32), act.title, font=font(30, True), fill=TEXT)
    badge, col = {14: ('OFFBOARD', TEAL), 5: ('FAILSAFE: RETURN', RED), 18: ('LANDING', AMBER), 4: ('HOLD', MUTED)}.get(int(nav), ('PX4', MUTED))
    f = font(24, True)
    w = d.textlength(badge, font=f) + 36
    x1 = W - 24
    rrect(d, (x1 - w, 22, x1, 66), 22, fill=col)
    d.text((x1 - w + 18, 26), badge, font=f, fill=(10, 16, 24))
    clock = f't = {t:5.1f} s' + (f'    {speed:g}x fast-forward' if speed > 1 else (f'    {speed:g}x SLOW MOTION' if speed < 1 else '    real time'))
    d.text((x1 - w - 20 - d.textlength(clock, font=font(18)), 32), clock, font=font(18), fill=MUTED)


def find_drone(frame, prev):
    cx, cy = prev
    x0, x1 = max(0, int(cx) - 170), min(VW, int(cx) + 170)
    y0, y1 = max(0, int(cy) - 120), min(VH, int(cy) + 120)
    c = frame[y0:y1, x0:x1].astype(np.int16)
    mask = (c[..., 0] < 80) & (c[..., 1] < 80) & (c[..., 2] < 88)
    if mask.sum() > 25:
        ys, xs = np.nonzero(mask)
        return (0.6 * cx + 0.4 * (x0 + xs.mean()), 0.6 * cy + 0.4 * (y0 + ys.mean()))
    return prev


def draw_video_overlay(d, act, t, i, ring, tt, speed):
    x, y = VX + ring[0], VY + ring[1]
    r = 50 + 4 * math.sin(tt * 9)
    for k, a in ((10, 40), (6, 80)):
        d.ellipse((x - r - k, y - r - k, x + r + k, y + r + k), outline=(*TEAL, a), width=3)
    d.ellipse((x - r, y - r, x + r, y + r), outline=TEAL, width=4)
    d.line((x + r * 0.7, y - r * 0.7, x + r + 34, y - r - 26), fill=TEAL, width=2)
    lines = [f'x500  ALT {max(act.alt[i], 0.0):4.1f} m', f'SPEED {act.sph[i]:3.1f} m/s']
    bx, by = x + r + 34, y - r - 70
    rrect(d, (bx, by, bx + 220, by + 66), 8, fill=(13, 21, 32, 215), outline=TEAL, width=2)
    d.text((bx + 10, by + 5), lines[0], font=font(19, True), fill=TEXT)
    d.text((bx + 10, by + 33), lines[1], font=font(19), fill=BLUE)
    tag = PHASE_TAG.get(act.stage[i], '')
    rrect(d, (VX + 14, VY + 14, VX + 34 + d.textlength(tag, font=font(22, True)), VY + 56), 8, fill=(13, 21, 32, 215), outline=BORDER, width=2)
    d.text((VX + 24, VY + 18), tag, font=font(22, True), fill=AMBER)
    if act.kind == 'gap' and act.gap_on and act.gap_on <= t:
        silent = act.since[i]
        in_gap = (act.gap_off is None or t < act.gap_off) and not (act.exit_t and t >= act.exit_t)
        if in_gap:
            col = AMBER if silent < 0.7 else RED
            rrect(d, (VX + 300, VY + 14, VX + VW - 300, VY + 66), 10, fill=(*col, 235))
            msg = f'SETPOINT STREAM PAUSED   {silent:4.2f} s   /   PX4 timeout {TIMEOUT_S:g} s'
            d.text((VX + VW / 2 - d.textlength(msg, font=font(24, True)) / 2, VY + 22), msg, font=font(24, True), fill=(10, 16, 24))
        elif act.exit_t and t >= act.exit_t and t < act.exit_t + 4.5:
            flash = 0.55 + 0.45 * math.sin(tt * 14)
            d.rectangle((VX, VY, VX + VW, VY + VH), outline=(*RED, int(255 * flash)), width=8)
            rrect(d, (VX + 250, VY + 14, VX + VW - 250, VY + 70), 10, fill=(*RED, 240))
            msg = 'OFFBOARD LOST  ->  RETURN-TO-LAUNCH'
            d.text((VX + VW / 2 - d.textlength(msg, font=font(28, True)) / 2, VY + 20), msg, font=font(28, True), fill=(255, 255, 255))
        elif act.gap_off and not act.exit_t and act.gap_off <= t < act.gap_off + 4.0:
            rrect(d, (VX + 300, VY + 14, VX + VW - 300, VY + 66), 10, fill=(*TEAL, 235))
            msg = 'STREAM RESUMED  -  OFFBOARD MODE KEPT'
            d.text((VX + VW / 2 - d.textlength(msg, font=font(24, True)) / 2, VY + 22), msg, font=font(24, True), fill=(10, 16, 24))


def draw_gamepad(d, act, i, t):
    df = act.df
    ox, oy = CX, CY
    d.text((ox + 18, oy + 12), 'CONTROLLER INPUT', font=font(14, True), fill=MUTED)
    d.text((ox + 18, oy + 30), 'scripted joystick messages (/joy) replayed as human-like stick motion', font=font(12), fill=MUTED)
    rrect(d, (ox + 36, oy + 78, ox + CW - 36, oy + 300), 70, fill=(28, 42, 64), outline=BORDER, width=2)
    rrect(d, (ox + 62, oy + 60, ox + 190, oy + 84), 10, fill=(40, 58, 86))
    rrect(d, (ox + CW - 190, oy + 60, ox + CW - 62, oy + 84), 10, fill=(40, 58, 86))
    yaw, up, right, fwd = (float(df.ax_yaw[i]), float(df.ax_up[i]), float(df.ax_right[i]), float(df.ax_fwd[i]))
    sticks = [((ox + 160, oy + 195), yaw, up, 'LEFT STICK', 'yaw  /  climb'),
              ((ox + CW - 160, oy + 195), right, fwd, 'RIGHT STICK', 'sideways  /  forward')]
    for (cx, cy), ah, av, name, sub in sticks:
        active = abs(ah) > 0.05 or abs(av) > 0.05
        d.ellipse((cx - 62, cy - 62, cx + 62, cy + 62), fill=(16, 26, 42), outline=TEAL if active else BORDER, width=4 if active else 2)
        d.line((cx - 62, cy, cx + 62, cy), fill=BORDER, width=1)
        d.line((cx, cy - 62, cx, cy + 62), fill=BORDER, width=1)
        nx, ny = cx - ah * 36, cy - av * 36          # ROS joy: left / up are positive
        d.line((cx, cy, nx, ny), fill=TEAL if active else BORDER, width=5)
        d.ellipse((nx - 25, ny - 25, nx + 25, ny + 25), fill=TEAL if active else (70, 92, 122), outline=(255, 255, 255) if active else MUTED, width=3)
        d.text((cx - d.textlength(name, font=font(14, True)) / 2, cy + 70), name, font=font(14, True), fill=TEXT if active else MUTED)
        d.text((cx - d.textlength(sub, font=font(12)) / 2, cy + 88), sub, font=font(12), fill=MUTED)
    mx, my = ox + CW // 2, oy + 175
    pressed = int(df.btn_land[i]) == 1
    for name, dx, dy, colr in (('Y', 0, -34, AMBER), ('X', -34, 0, BLUE), ('B', 34, 0, RED), ('A', 0, 34, TEAL)):
        on = pressed and name == 'A'
        d.ellipse((mx + dx - 19, my + dy - 19, mx + dx + 19, my + dy + 19), fill=colr if on else (36, 52, 78), outline=colr, width=3)
        d.text((mx + dx - 7, my + dy - 13), name, font=font(20, True), fill=(10, 16, 24) if on else colr)
    d.text((mx - 20, my + 56), 'LAND' if pressed else 'land', font=font(14, True), fill=TEAL if pressed else MUTED)
    n, e, dn = act.ned[i]
    yr = act.yaw_rate[i]
    d.text((ox + 18, oy + 322), 'Setpoint sent to PX4 (NED):', font=font(14, True), fill=MUTED)
    txt = f'N {n:+5.2f}   E {e:+5.2f}   D {dn:+5.2f} m/s     yaw rate {yr:+4.2f} rad/s'
    d.text((ox + 18, oy + 344), txt, font=font(17, True), fill=TEXT)
    d.text((ox + 18, oy + 366), 'deadzone 0.1, limits 2 / 1 m/s, body frame -> NED via heading', font=font(12), fill=MUTED)


def draw_gauge(d, act, i, t):
    ox, oy = CX, CY
    d.text((ox + 18, oy + 12), 'EXPERIMENT', font=font(14, True), fill=MUTED)
    d.text((ox + 18, oy + 34), f'Pause the setpoint stream for {act.gap:g} s', font=font(26, True), fill=TEXT)
    d.text((ox + 18, oy + 72), 'Autonomous script, velocity square, one injected gap', font=font(14), fill=MUTED)
    silent = float(act.since[i])
    lost = act.exit_t is not None and t >= act.exit_t
    landing = act.stage[i] == 'land' and not lost
    d.text((ox + 18, oy + 112), 'TIME SINCE LAST SETPOINT', font=font(14, True), fill=MUTED)
    shown = '--' if (lost or landing) else f'{silent:4.2f} s'
    col = TEAL if silent < 0.5 else (AMBER if silent < 1.0 else RED)
    if lost:
        col = RED
    elif landing:
        col = MUTED
    d.text((ox + 18, oy + 132), shown, font=font(78, True), fill=col)
    bx0, bx1, by = ox + 18, ox + CW - 18, oy + 250
    rrect(d, (bx0, by, bx1, by + 30), 8, fill=(15, 26, 42), outline=BORDER, width=2)
    frac = 0.0 if landing else (min(1.0, silent / 1.2) if not lost else 1.0)
    if frac > 0.01:
        rrect(d, (bx0 + 2, by + 2, bx0 + 2 + (bx1 - bx0 - 4) * frac, by + 28), 6, fill=col)
    tx = bx0 + (bx1 - bx0) * (TIMEOUT_S / 1.2)
    d.line((tx, by - 10, tx, by + 40), fill=RED, width=4)
    d.text((tx - 62, by + 42), 'PX4 timeout 1.0 s', font=font(14, True), fill=RED)
    d.text((bx0, by + 42), '0 s', font=font(13), fill=MUTED)
    if lost:
        msg, c2 = 'FAILSAFE TRIGGERED: offboard lost', RED
    elif landing:
        msg, c2 = 'Landing: stream no longer needed', MUTED
    elif act.gap_off and t >= act.gap_off:
        msg, c2 = 'Offboard mode kept', TEAL
    elif act.gap_on and t >= act.gap_on:
        msg, c2 = 'Counting silence...', AMBER
    else:
        msg, c2 = 'Streaming normally at 20 Hz', TEAL
    d.text((ox + 18, oy + 338), msg, font=font(22, True), fill=c2)


def draw_caption(d, cap):
    title, body, tone = cap
    col = {'info': BLUE, 'warn': AMBER, 'alert': RED, 'ok': TEAL}[tone]
    d.rectangle((KX, KY + 44, KX + 6, KY + KH - 18), fill=col)
    d.text((KX + 22, KY + 44), title, font=font(25, True), fill=col)
    y = KY + 84
    for line in wrap(d, body, font(18), KW - 44):
        d.text((KX + 22, y), line, font=font(18), fill=TEXT)
        y += 25
    rrect(d, (KX + 18, KY + KH - 66, KX + KW - 18, KY + KH - 14), 8, fill=(15, 26, 42), outline=BORDER, width=2)
    d.text((KX + 30, KY + KH - 60), 'THE RULE', font=font(12, True), fill=TEAL)
    d.text((KX + 30, KY + KH - 41), 'Stream before the mode switch; never go silent for 1 s (stream faster than 2 Hz).', font=font(13), fill=TEXT)


def plot_panel(arr, plots, extents, act, t):
    t0, t1 = extents
    for (px, py, pw, ph), (full, dim) in zip(PLOTS, plots):
        axl, axr = int(AX[0] * pw), int((AX[0] + AX[2]) * pw)
        cx = int(axl + (min(max(t, t0), t1) - t0) / (t1 - t0) * (axr - axl))
        img = dim.copy()
        img[:, axl:cx] = full[:, axl:cx]
        img[48:216, cx:cx + 2] = (255, 255, 255)
        arr[py:py + ph, px:px + pw] = img


# ----------------------------------------------------------------------------------------- cards


def card(title, subtitle, bullets=(), foot='', accent=TEAL, big=True):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    d.rectangle((0, 0, 14, H), fill=accent)
    d.text((110, 250 if bullets else 380), title, font=font(78 if big else 60, True), fill=TEXT)
    y = 360 if bullets else 500
    d.text((112, y), subtitle, font=font(34), fill=accent)
    yy = y + 90
    for b in bullets:
        lines = wrap(d, b, font(30), W - 330)
        d.ellipse((116, yy + 11, 130, yy + 25), fill=accent)
        for k, line in enumerate(lines):
            d.text((152, yy + k * 38), line, font=font(30), fill=TEXT)
        yy += 38 * len(lines) + 26
    if foot:
        d.text((112, H - 70), foot, font=font(20), fill=MUTED)
    return np.array(img)


def fade(arr, k, n):
    a = min(1.0, k / (0.35 * FPS), (n - k) / (0.35 * FPS))
    return (arr.astype(np.float32) * a + np.array(BG) * (1 - a)).astype(np.uint8)


# ----------------------------------------------------------------------------------------- main


class Reader:
    def __init__(self, path, vstart):
        self.p = subprocess.Popen([FF, '-v', 'error', '-ss', f'{vstart:.3f}', '-i', path, '-vf', f'fps={FPS},crop=1624:912:32:0,scale={VW}:{VH}:flags=bicubic',
                                   '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE, bufsize=VW * VH * 3 * 8)
        self.idx, self.vstart, self.cur = 0, vstart, None

    def get(self, vt):
        target = int(round((vt - self.vstart) * FPS))
        while self.idx <= target:
            buf = self.p.stdout.read(VW * VH * 3)
            if len(buf) < VW * VH * 3:
                break
            self.cur = np.frombuffer(buf, np.uint8).reshape(VH, VW, 3)
            self.idx += 1
        return self.cur

    def close(self):
        self.p.stdout.close()
        self.p.kill()


def render_act(act, bg, writer, shots=None):
    plots, extents = make_plots(act)
    reader = None
    ring = (VW * 0.5, VH * 0.46)
    times = act.times()
    vstart = max(0.0, times[0][0] + act.off)
    reader = Reader(os.path.join(RAW, f'act_{act.key}.mp4'), vstart)
    n = len(times)
    for k, (t, speed) in enumerate(times):
        frame = reader.get(t + act.off)
        if frame is None:
            break
        i = act.idx(t)
        ring = find_drone(frame, ring)
        arr = bg.copy()
        arr[VY:VY + VH, VX:VX + VW] = frame
        plot_panel(arr, plots, extents, act, t)
        img = Image.fromarray(arr)
        d = ImageDraw.Draw(img, 'RGBA')
        draw_header(d, act, t, speed, act.nav[i])
        draw_video_overlay(d, act, t, i, ring, k / FPS, speed)
        (draw_gamepad if act.kind == 'teleop' else draw_gauge)(d, act, i, t)
        draw_caption(d, caption(act, t))
        out = np.array(img)
        if shots is not None:
            if k == int(n * shots):
                return out
        else:
            writer.stdin.write(fade(out, k, n).tobytes() if k < 10 or n - k < 10 else out.tobytes())
    reader.close()
    return None


def main():
    shots_dir = sys.argv[sys.argv.index('--shots') + 1] if '--shots' in sys.argv else None
    out_path = sys.argv[1] if shots_dir is None else None
    acts = [Act('A_teleop', 'teleop', 'ACT 1  -  Flying with a joystick', 'Every stick motion becomes a velocity setpoint, 20 times per second.'),
            Act('B_gap0.6', 'gap', 'ACT 2  -  The test: pause the stream for 0.6 s', 'PX4 waits 1.0 s before it gives up. Does it survive?', gap=0.6),
            Act('C_gap1.5', 'gap', 'ACT 3  -  Pause it for 1.5 s: the failsafe', 'Longer than the timeout. What does PX4 do?', gap=1.5)]
    for a in acts:
        a.detect_offset()
    bg = static_background()
    title = card('PX4 offboard control', 'Why the command stream must never stop',
                 ['A ROS 2 node (the companion computer) flies a simulated x500 through micro-XRCE-DDS and PX4',
                  'PX4 only obeys while setpoints keep arriving: go silent for 1.0 s and it takes the drone back',
                  'Real Gazebo footage, flight logs and measurements, all in sync'],
                 foot='Simulation only (PX4 SITL + Gazebo Harmonic). Details and caveats in FINDINGS.md')
    outro = card('What the measurements show', 'PX4 SITL, 2-3 runs per condition',
                 ['With the timeout set to 0.5 / 1.0 / 2.0 s, PX4 left offboard 0.57 / 1.05 / 2.0 s after the stream stopped',
                  'Single gaps up to 0.9 s were tolerated; 1.1 s and longer lost offboard mode (9 of 9 runs)',
                  'A steady 1.5 Hz stream was safe, 1 Hz was not; tracking error grows as the rate falls (0.47 to 0.59 m/s)',
                  'The timeout runs in PX4 time, so thresholds measured in wall-clock Hz move with simulation speed'],
                 foot='Simulation only; sweep thresholds are indicative (see FINDINGS.md, section 3). Repo: px4-offboard-timing-study (private)', accent=ORANGE, big=False)
    if shots_dir:
        os.makedirs(shots_dir, exist_ok=True)
        Image.fromarray(title).save(os.path.join(shots_dir, 'card_title.png'))
        Image.fromarray(outro).save(os.path.join(shots_dir, 'card_outro.png'))
        for a in acts:
            for frac in (0.42, 0.80):
                frame = render_act(a, bg, None, shots=frac)
                Image.fromarray(frame).save(os.path.join(shots_dir, f'shot_{a.key}_{int(frac * 100)}.png'))
                print('shot', a.key, frac)
        return
    writer = subprocess.Popen([FF, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
                               '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', out_path],
                              stdin=subprocess.PIPE)
    for k in range(int(4.5 * FPS)):
        writer.stdin.write(fade(title, k, int(4.5 * FPS)).tobytes())
    for a in acts:
        slate = card(a.title, a.goal, foot='', big=False)
        n = int(2.6 * FPS)
        for k in range(n):
            writer.stdin.write(fade(slate, k, n).tobytes())
        render_act(a, bg, writer)
        print('rendered', a.key, flush=True)
    for k in range(int(11 * FPS)):
        writer.stdin.write(fade(outro, k, int(11 * FPS)).tobytes())
    writer.stdin.close()
    writer.wait()
    print('done', out_path)


if __name__ == '__main__':
    main()
