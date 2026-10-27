"""Minimal monospace restyle of the showcase video, in four palettes.

Usage: python compose_minimal.py previews <out_dir>        short clip + 3 stills for each style
       python compose_minimal.py full <style> <out.mp4>    full video in one style
Styles: A_mono_black, B_mono_paper, C_amber, D_swiss.
Data, sync and layout come from compose_video.py; this file only changes how it looks.
"""
import math
import os
import subprocess
import sys

import matplotlib
import numpy as np
from matplotlib import font_manager
from PIL import Image, ImageDraw, ImageFont

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

import compose_video as cv  # noqa: E402
from compose_video import (AX, CH, CW, CX, CY, FF, FPS, H, KH, KW, KX, KY, PLOTS, VH, VW, VX, VY, W, Act, Reader,  # noqa: E402
                           find_drone)

FONT_REG, FONT_BOLD = r'C:\Windows\Fonts\consola.ttf', r'C:\Windows\Fonts\consolab.ttf'
STYLES = {
    # name: (bg, fg, muted, accent, footage, alert_mode)
    'A_mono_black': ((0, 0, 0), (238, 238, 238), (118, 118, 118), None, 'gray', 'inverse'),
    'B_mono_paper': ((244, 244, 242), (14, 14, 14), (135, 135, 132), None, 'gray', 'inverse'),
    'C_amber': ((10, 10, 10), (226, 226, 222), (112, 112, 108), (255, 176, 0), 'gray', 'accent'),
    'D_swiss': ((240, 240, 236), (18, 18, 18), (140, 140, 134), (230, 54, 24), 'muted', 'accent'),
}
T = {}


def set_style(name):
    bg, fg, muted, accent, footage, alert = STYLES[name]
    T.update(name=name, bg=bg, fg=fg, muted=muted, accent=accent or fg, has_accent=accent is not None,
             footage=footage, alert=alert)
    for f in (FONT_REG, FONT_BOLD):
        font_manager.fontManager.addfont(f)
    plt.rcParams.update({'font.family': 'Consolas', 'font.size': 8.5, 'hatch.linewidth': 0.7,
                         'text.color': hx(fg), 'axes.labelcolor': hx(muted), 'xtick.color': hx(muted),
                         'ytick.color': hx(muted), 'axes.edgecolor': hx(fg)})


def hx(c):
    return '#%02x%02x%02x' % tuple(c)


_f = {}


def font(size, bold=False):
    if (size, bold) not in _f:
        _f[(size, bold)] = ImageFont.truetype(FONT_BOLD if bold else FONT_REG, size)
    return _f[(size, bold)]


def text(d, xy, s, size=16, bold=False, fill=None):
    d.text(xy, s, font=font(size, bold), fill=fill or T['fg'])


def tw(d, s, size, bold=False):
    return d.textlength(s, font=font(size, bold))


def wrap(d, s, size, maxw):
    lines, line = [], ''
    for w in s.split():
        trial = (line + ' ' + w).strip()
        if tw(d, trial, size) <= maxw:
            line = trial
        else:
            lines.append(line)
            line = w
    return lines + [line]


def box(d, xyxy, fill=None, outline=None, width=1):
    d.rectangle(xyxy, fill=fill, outline=outline, width=width)


def inverse_box(d, xyxy, s, size=18, bold=True, pad=8, accent=False):
    """Filled block with background-coloured text: the minimal way to shout."""
    x0, y0, x1, y1 = xyxy
    fill = T['accent'] if (accent and T['has_accent']) else T['fg']
    box(d, xyxy, fill=fill)
    text(d, (x0 + pad, y0 + (y1 - y0 - size * 1.25) / 2), s, size, bold, fill=T['bg'])


# ------------------------------------------------------------------------------------------ footage


def style_frame(fr):
    lum = fr[..., 0] * 0.299 + fr[..., 1] * 0.587 + fr[..., 2] * 0.114
    if T['footage'] == 'gray':
        g = np.clip((lum - 128) * 1.12 + 128, 0, 255)
        return np.repeat(g[..., None], 3, axis=2).astype(np.uint8)
    out = fr.astype(np.float32) * 0.35 + lum[..., None] * 0.65
    return np.clip(out, 0, 255).astype(np.uint8)


# ------------------------------------------------------------------------------------------ captions


def caption(act, t):
    i = act.idx(t)
    st = act.stage[i]
    if act.kind == 'teleop':
        if t < act.t_arm + 0.3:
            return ('stream first, then switch mode', 'setpoints go out at 20 Hz before the node asks PX4 for offboard mode. without a stream PX4 refuses the switch.', 0)
        if st == 'idle' and t < 13.5:
            return ('arm, offboard, climb to 5 m', 'PX4 follows the ROS 2 node instead of a pilot. position setpoint 5 m up, streamed continuously.', 0)
        tab = {
            'idle': ('sticks centred: hold', 'zero velocity setpoint. the stream keeps flowing, so PX4 stays in offboard.', 0),
            'forward': ('right stick forward', 'deadzone, 2 m/s limit, then body frame to NED with the current heading (the drone faces east).', 0),
            'right': ('right stick sideways', 'same pipeline. right of an east-facing drone is south. this sign was inverted in my first version.', 1),
            'climb': ('left stick up: climb', 'vertical speed limited to 1 m/s, sent as NED down-velocity (negative = up).', 0),
            'descend': ('left stick down: descend', 'same limit, other direction. watch the altitude plot.', 0),
            'carve': ('forward + yaw: carving turn', 'both sticks at once. the body-to-NED conversion follows the changing heading.', 0),
            'land': ('button A: land', 'land command sent. PX4 lands and disarms. the stream is no longer needed.', 0),
        }
        return tab.get(st, tab['idle'])
    if st == 'warmup':
        return ('stream first, then switch mode', 'setpoints go out at 20 Hz before the script asks PX4 for offboard mode.', 0)
    if st == 'takeoff':
        return ('arm, offboard, climb to 5 m', 'position setpoint 5 m up, streamed continuously.', 0)
    if st == 'hold':
        return ('hold position', 'nothing changes but the stream keeps flowing. the heartbeat must not stop.', 0)
    if st == 'turn':
        return ('yaw turn, 90 deg', 'still 20 Hz: one heartbeat and one setpoint every 50 ms.', 0)
    g = f'{act.gap:g} s'
    if act.exit_t and act.exit_t <= t < act.exit_t + 4.5:
        return ('timeout: PX4 leaves offboard', 'after 1.0 s of silence PX4 assumes the companion computer died and takes over: return-to-launch.', 2)
    if st == 'track':
        gap_end = act.gap_off if act.gap_off else float('inf')
        if act.gap_on and t < act.gap_on:
            return ('velocity square at 20 Hz', 'longest silence so far 0.05 s. PX4 waits 1.0 s (COM_OF_LOSS_T): 20x margin.', 0)
        if t < gap_end and not (act.exit_t and t >= act.exit_t):
            return (f'test: stream paused for {g}', 'no message arrives. PX4 counts the silence against its 1.0 s timeout. watch the gauge and the stream plot.', 1)
        return ('stream is back: offboard kept', f'{g} of silence is under the 1.0 s timeout, so no failsafe. only the command that changed during the gap was missed.', 0)
    if st == 'settle':
        return ('square finished', 'back to steady 20 Hz. the drone settles.', 0)
    if act.exit_t:
        return ('failsafe landing', 'PX4 is no longer in offboard, so the script just commands a landing. test over.', 2)
    return ('landing', 'land command sent, script stops streaming. in land mode silence is harmless.', 0)


TAGS = {'warmup': 'streaming', 'takeoff': 'takeoff', 'hold': 'hold', 'turn': 'yaw turn', 'track': 'velocity square',
        'settle': 'settle', 'land': 'landing', 'idle': 'hold', 'forward': 'forward', 'right': 'sideways', 'climb': 'climb',
        'descend': 'descend', 'carve': 'carving turn'}


# ------------------------------------------------------------------------------------------ plots


def make_plots(act):
    ts = act.times()
    t0, t1 = ts[0][0], ts[-1][0]
    m = (act.t >= t0) & (act.t <= t1)
    t = act.t[m]
    fg, mu, ac, bg = hx(T['fg']), hx(T['muted']), hx(T['accent']), hx(T['bg'])
    figs = []

    def base(title, raster=False, sub=''):
        fig = plt.figure(figsize=(PLOTS[0][2] / 100, PLOTS[0][3] / 100), dpi=100, facecolor=bg)
        if raster:
            ax = fig.add_axes([AX[0], AX[1], AX[2], 0.40])
            rx = fig.add_axes([AX[0], 0.64, AX[2], 0.14], sharex=ax)
            rx.set_facecolor(bg)
            rx.set_yticks([])
            rx.tick_params(labelbottom=False, length=0)
            for k in ('top', 'right', 'left'):
                rx.spines[k].set_visible(False)
            rx.spines['bottom'].set_color(mu)
        else:
            ax, rx = fig.add_axes(list(AX)), None
        ax.set_facecolor(bg)
        for k in ('top', 'right'):
            ax.spines[k].set_visible(False)
        ax.spines['left'].set_color(mu)
        ax.spines['bottom'].set_color(mu)
        ax.tick_params(length=3, width=0.6)
        fig.text(0.02, 0.93, title, fontsize=9.5, fontweight='bold', color=fg, va='center')
        fig.text(0.02, 0.855, sub, fontsize=7.5, color=mu, va='center')
        ax.set_xlim(t0, t1)
        return fig, ax, rx

    def bands(ax):
        nav = act.nav[m]
        start = 0
        for i in range(1, len(nav) + 1):
            if i == len(nav) or nav[i] != nav[start]:
                a, b = t[start], t[min(i, len(t) - 1)]
                n = int(nav[start])
                if n == 5:
                    ax.axvspan(a, b, facecolor='none', edgecolor=ac, hatch='////', lw=0)
                elif n == 18:
                    ax.axvspan(a, b, color=mu, alpha=0.14, lw=0)
                start = i

    fig, ax, _ = base('speed: command vs measured  [m/s]', sub='dashed = command, solid = measured')
    bands(ax)
    ax.plot(t, act.cmd_h[m], color=fg, lw=0.9, ls=(0, (4, 3)))
    ax.plot(t, act.sph[m], color=fg, lw=1.7)
    ax.set_xlabel('time [s]')
    figs.append(fig)

    fig, ax, _ = base('altitude  [m]', sub='hatched = failsafe, grey = landing')
    bands(ax)
    ax.plot(t, act.alt[m], color=fg, lw=1.7)
    ax.set_ylim(-0.2, max(7.0, act.alt[m].max() + 1.0))
    ax.set_xlabel('time [s]')
    figs.append(fig)

    fig, ax, rx = base('setpoint stream, silence vs timeout  [s]', raster=True)
    bands(ax)
    st = act.sent_t[(act.sent_t >= t0) & (act.sent_t <= t1)]
    rx.vlines(st, 0, 1, color=fg, lw=0.5)
    rx.set_ylim(0, 1)
    rx.text(0.0, 1.35, 'one tick = one message', transform=rx.transAxes, fontsize=7, color=mu, va='bottom')
    ax.plot(t, np.minimum(act.since[m], 1.5), color=fg, lw=1.6)
    ax.axhline(1.0, color=ac, lw=1.0, ls=(0, (4, 3)))
    ax.text(t0 + 0.012 * (t1 - t0), 1.04, 'PX4 timeout 1.0 s', color=ac, fontsize=7.5, fontweight='bold')
    ax.set_ylim(0, 1.5)
    ax.set_xlabel('time [s]')
    if act.exit_t:
        ax.axvline(act.exit_t, color=ac, lw=1.4)
        ax.text(act.exit_t - 0.012 * (t1 - t0), 1.35, 'failsafe', color=ac, fontsize=7.5, fontweight='bold', ha='right')
    figs.append(fig)
    out = []
    for f in figs:
        f.canvas.draw()
        full = np.asarray(f.canvas.buffer_rgba())[..., :3].copy()
        plt.close(f)
        dim = full.copy()
        c0, c1 = int(AX[0] * PLOTS[0][2]), int((AX[0] + AX[2]) * PLOTS[0][2])
        dim[54:212, c0:c1] = (full[54:212, c0:c1] * 0.32 + np.array(T['bg']) * 0.68).astype(np.uint8)
        out.append((full, dim))
    return out, (t0, t1)


def plot_panel(arr, plots, extents, t):
    t0, t1 = extents
    for (px, py, pw, ph), (full, dim) in zip(PLOTS, plots):
        axl, axr = int(AX[0] * pw), int((AX[0] + AX[2]) * pw)
        cx = int(axl + (min(max(t, t0), t1) - t0) / (t1 - t0) * (axr - axl))
        img = dim.copy()
        img[:, axl:cx] = full[:, axl:cx]
        img[48:216, cx:cx + 1] = T['accent']
        arr[py:py + ph, px:px + pw] = img


# ------------------------------------------------------------------------------------------ drawing


def background():
    img = Image.new('RGB', (W, H), T['bg'])
    d = ImageDraw.Draw(img)
    for x0, y0, x1, y1 in ((VX - 1, VY - 1, VX + VW, VY + VH), (CX, CY, CX + CW, CY + CH), (KX, KY, KX + KW, KY + KH)):
        box(d, (x0, y0, x1, y1), outline=T['muted'])
    text(d, (KX + 14, KY + 10), 'log', 13, fill=T['muted'])
    text(d, (VX, H - 22), 'real gazebo footage (gpu rendered) / px4 sitl / simulation only / see FINDINGS.md', 12, fill=T['muted'])
    return np.array(img)


def header(d, act, t, speed, nav):
    text(d, (24, 12), 'px4 offboard control', 14, fill=T['muted'])
    text(d, (24, 32), act.title, 26, True)
    label = {14: 'OFFBOARD', 5: 'FAILSAFE: RETURN', 18: 'LANDING', 4: 'HOLD'}.get(int(nav), 'PX4')
    s = f'[ {label} ]'
    w = tw(d, s, 24, True)
    x1 = W - 24
    if int(nav) == 5:
        inverse_box(d, (x1 - w - 16, 22, x1, 62), label, 22, accent=True)
        x1 -= w + 16
    else:
        text(d, (x1 - w, 28), s, 24, True)
        x1 -= w
    clock = f't={t:5.1f}s ' + (f'{speed:g}x fast-forward' if speed > 1 else (f'{speed:g}x slow motion' if speed < 1 else 'real time'))
    text(d, (x1 - 24 - tw(d, clock, 15), 34), clock, 15, fill=T['muted'])


def video_overlay(d, act, t, i, ring, tt):
    x, y = VX + ring[0], VY + ring[1]
    r = 46
    col = T['accent']
    d.ellipse((x - r, y - r, x + r, y + r), outline=col, width=2)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
        d.line((x + dx * r, y + dy * r, x + dx * (r + 12), y + dy * (r + 12)), fill=col, width=2)
    lines = ['x500', f'alt {max(act.alt[i], 0):4.1f} m', f'spd {act.sph[i]:4.1f} m/s']
    bx, by = x + r + 20, y - r - 14
    box(d, (bx, by, bx + 190, by + 76), fill=T['bg'], outline=col)
    for k, s in enumerate(lines):
        text(d, (bx + 10, by + 6 + k * 22), s, 16, k == 0)
    tag = TAGS.get(act.stage[i], '')
    if act.exit_t and act.exit_t <= t < act.exit_t + 4.5:
        tag = 'failsafe return'
    s = f'[ {tag} ]'
    box(d, (VX + 12, VY + 12, VX + 28 + tw(d, s, 18, True), VY + 44), fill=T['bg'], outline=T['muted'])
    text(d, (VX + 20, VY + 17), s, 18, True)
    if act.kind != 'gap' or not act.gap_on or act.gap_on > t:
        return
    silent = act.since[i]
    in_gap = (act.gap_off is None or t < act.gap_off) and not (act.exit_t and t >= act.exit_t)
    bar = (VX + 260, VY + 12, VX + VW - 260, VY + 56)
    if in_gap:
        s = f'STREAM PAUSED {silent:4.2f}s / timeout 1.0s'
        if silent >= 0.7:
            inverse_box(d, bar, s, 20, accent=True)
        else:
            box(d, bar, fill=T['bg'], outline=T['fg'], width=2)
            text(d, (bar[0] + 14, bar[1] + 10), s, 20, True)
    elif act.exit_t and act.exit_t <= t < act.exit_t + 4.5:
        if int(tt * 4) % 2 == 0:
            box(d, (VX, VY, VX + VW - 1, VY + VH - 1), outline=T['accent'], width=8)
        inverse_box(d, (VX + 250, VY + 12, VX + VW - 250, VY + 62), 'OFFBOARD LOST -> RETURN-TO-LAUNCH', 24, accent=True)
    elif act.gap_off and not act.exit_t and act.gap_off <= t < act.gap_off + 4.0:
        box(d, bar, fill=T['bg'], outline=T['fg'], width=2)
        text(d, (bar[0] + 14, bar[1] + 10), 'STREAM RESUMED - OFFBOARD KEPT', 20, True)


def gamepad(d, act, i):
    df = act.df
    yaw, up, right, fwd = (float(df.ax_yaw[i]), float(df.ax_up[i]), float(df.ax_right[i]), float(df.ax_fwd[i]))
    text(d, (CX + 14, CY + 10), 'input', 13, fill=T['muted'])
    text(d, (CX + 14, CY + 30), 'scripted /joy messages, replayed as stick motion', 12, fill=T['muted'])
    sticks = [((CX + 160, CY + 175), yaw, up, 'left stick', 'yaw / climb'), ((CX + CW - 160, CY + 175), right, fwd, 'right stick', 'sideways / forward')]
    for (cx, cy), ah, av, name, sub in sticks:
        on = abs(ah) > 0.05 or abs(av) > 0.05
        d.ellipse((cx - 62, cy - 62, cx + 62, cy + 62), outline=T['fg'] if on else T['muted'], width=2)
        d.line((cx - 62, cy, cx + 62, cy), fill=T['muted'], width=1)
        d.line((cx, cy - 62, cx, cy + 62), fill=T['muted'], width=1)
        nx, ny = cx - ah * 38, cy - av * 38
        d.line((cx, cy, nx, ny), fill=T['accent'] if on else T['muted'], width=2)
        d.ellipse((nx - 14, ny - 14, nx + 14, ny + 14), fill=T['accent'] if on else T['bg'], outline=T['fg'] if on else T['muted'], width=2)
        text(d, (cx - tw(d, name, 15, True) / 2, cy + 72), name, 15, True, T['fg'] if on else T['muted'])
        text(d, (cx - tw(d, sub, 12) / 2, cy + 92), sub, 12, fill=T['muted'])
    mx, my = CX + CW // 2, CY + 140
    pressed = int(df.btn_land[i]) == 1
    for name, dx, dy in (('Y', 0, -30), ('X', -30, 0), ('B', 30, 0), ('A', 0, 30)):
        on = pressed and name == 'A'
        d.rectangle((mx + dx - 14, my + dy - 14, mx + dx + 14, my + dy + 14), fill=T['accent'] if on else T['bg'], outline=T['fg'] if on else T['muted'], width=2)
        text(d, (mx + dx - 6, my + dy - 11), name, 18, True, T['bg'] if on else T['muted'])
    text(d, (mx - 24, my + 50), 'LAND' if pressed else 'land', 13, pressed, T['fg'] if pressed else T['muted'])
    n, e, dn = act.ned[i]
    text(d, (CX + 14, CY + 300), 'setpoint to PX4 (NED, m/s)', 13, fill=T['muted'])
    text(d, (CX + 14, CY + 320), f'N {n:+5.2f}  E {e:+5.2f}  D {dn:+5.2f}  yaw {act.yaw_rate[i]:+4.2f} rad/s', 17, True)
    text(d, (CX + 14, CY + 350), 'deadzone 0.1, limits 2 / 1 m/s, body frame to NED', 12, fill=T['muted'])


def gauge(d, act, i, t):
    silent = float(act.since[i])
    lost = act.exit_t is not None and t >= act.exit_t
    landing = act.stage[i] == 'land' and not lost
    text(d, (CX + 14, CY + 10), 'experiment', 13, fill=T['muted'])
    text(d, (CX + 14, CY + 52), f'pause the stream for {act.gap:g} s', 24, True)
    text(d, (CX + 14, CY + 90), 'autonomous script, velocity square, one gap', 12, fill=T['muted'])
    text(d, (CX + 14, CY + 126), 'silence since last setpoint', 13, fill=T['muted'])
    shown = '--' if (lost or landing) else f'{silent:4.2f} s'
    hot = lost or (silent >= 0.7 and not landing)
    text(d, (CX + 14, CY + 146), shown, 76, True, T['accent'] if hot else (T['muted'] if landing else T['fg']))
    n = 36
    frac = 0 if landing else (1.0 if lost else min(1.0, silent / 1.2))
    fill_n = int(round(frac * n))
    tick = int(round(n / 1.2))
    bar = ''.join('█' if k < fill_n else '·' for k in range(n))
    text(d, (CX + 14, CY + 250), '[' + bar + ']', 17, True, T['accent'] if hot else T['fg'])
    chw = tw(d, bar, 17, True) / n
    cx_tick = CX + 14 + tw(d, '[', 17, True) + tick * chw + chw / 2
    text(d, (cx_tick - tw(d, '^', 13, True) / 2, CY + 272), '^', 13, True, T['accent'])
    text(d, (cx_tick + 12, CY + 272), 'timeout 1.0 s', 13, True, T['accent'])
    if lost:
        msg, c = 'FAILSAFE TRIGGERED: offboard lost', T['accent']
    elif landing:
        msg, c = 'landing: stream no longer needed', T['muted']
    elif act.gap_off and t >= act.gap_off:
        msg, c = 'offboard kept', T['fg']
    elif act.gap_on and t >= act.gap_on:
        msg, c = 'counting silence...', T['accent']
    else:
        msg, c = 'streaming normally at 20 Hz', T['fg']
    text(d, (CX + 14, CY + 330), msg, 19, True, c)


def captions(d, cap):
    title, body, level = cap
    c = T['accent'] if level else T['fg']
    d.line((KX + 14, KY + 36, KX + KW - 14, KY + 36), fill=T['muted'], width=1)
    text(d, (KX + 14, KY + 46), title, 22, True, c)
    y = KY + 84
    for line in wrap(d, body, 16, KW - 28):
        text(d, (KX + 14, y), line, 16)
        y += 22
    d.line((KX + 14, KY + KH - 52, KX + KW - 14, KY + KH - 52), fill=T['muted'], width=1)
    text(d, (KX + 14, KY + KH - 44), 'rule', 12, fill=T['muted'])
    text(d, (KX + 14, KY + KH - 28), 'stream before the mode switch. never go silent for 1 s.', 13)


def card(title, sub, bullets=(), foot='', size=64):
    img = Image.new('RGB', (W, H), T['bg'])
    d = ImageDraw.Draw(img)
    y = 300 if bullets else 440
    text(d, (110, y), title, size, True)
    text(d, (112, y + size + 24), sub, 26, fill=T['muted'])
    yy = y + size + 100
    for b in bullets:
        lines = wrap(d, b, 24, W - 320)
        text(d, (112, yy), '-', 24, True)
        for k, line in enumerate(lines):
            text(d, (150, yy + k * 32), line, 24)
        yy += 32 * len(lines) + 22
    if foot:
        text(d, (112, H - 60), foot, 16, fill=T['muted'])
    return np.array(img)


# ------------------------------------------------------------------------------------------ rendering


def frame_at(act, bg, plots, extents, reader, t, speed, ring, tt):
    fr = reader.get(t + act.off)
    if fr is None:
        return None, ring
    i = act.idx(t)
    ring = find_drone(fr, ring)
    arr = bg.copy()
    arr[VY:VY + VH, VX:VX + VW] = style_frame(fr)
    plot_panel(arr, plots, extents, t)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    header(d, act, t, speed, act.nav[i])
    video_overlay(d, act, t, i, ring, tt)
    (gamepad(d, act, i) if act.kind == 'teleop' else gauge(d, act, i, t))
    captions(d, caption(act, t))
    return np.array(img), ring


def fade(arr, k, n):
    a = min(1.0, k / (0.3 * FPS), (n - k) / (0.3 * FPS))
    return (arr.astype(np.float32) * a + np.array(T['bg']) * (1 - a)).astype(np.uint8)


def segment_times(a, b, speed):
    out, t = [], a
    while t < b:
        out.append((t, speed))
        t += speed / FPS
    return out


def run_segments(act, bg, plots, extents, segs, sink, shots=None):
    """segs: list of lists of (t, speed). Writes frames to sink(arr) or returns the frame closest to each time in shots."""
    found = {}
    for times in segs:
        reader = Reader(os.path.join(cv.RAW, f'act_{act.key}.mp4'), max(0.0, times[0][0] + act.off))
        ring = (VW * 0.5, VH * 0.46)
        n = len(times)
        for k, (t, sp) in enumerate(times):
            out, ring = frame_at(act, bg, plots, extents, reader, t, sp, ring, k / FPS)
            if out is None:
                break
            if shots:
                for name, ts in shots.items():
                    if name not in found and abs(t - ts) <= sp / FPS:
                        found[name] = out
            if sink:
                sink(fade(out, k, n) if (k < 8 or n - k < 8) else out)
        reader.close()
    return found


def acts_list():
    acts = [Act('A_teleop', 'teleop', 'act 1 / flying with a joystick', ''),
            Act('B_gap0.6', 'gap', 'act 2 / pause the stream for 0.6 s', '', gap=0.6),
            Act('C_gap1.5', 'gap', 'act 3 / pause it for 1.5 s', '', gap=1.5)]
    for a in acts:
        a.detect_offset()
    return acts


def writer(path):
    return subprocess.Popen([FF, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-',
                             '-c:v', 'libx264', '-preset', 'fast', '-threads', '6', '-crf', '18', '-pix_fmt', 'yuv420p', path],
                            stdin=subprocess.PIPE)


def title_card():
    return card('px4 offboard control', 'why the command stream must never stop',
                ['a ROS 2 node flies a simulated x500 through micro-XRCE-DDS and PX4',
                 'PX4 obeys only while setpoints keep arriving. silent for 1.0 s and it takes the drone back',
                 'real gazebo footage, flight logs and measurements, in sync'],
                foot='simulation only (PX4 SITL + Gazebo Harmonic). details and caveats: FINDINGS.md')


def outro_card():
    return card('what the measurements show', 'PX4 SITL, 2-3 runs per condition',
                ['timeout set to 0.5 / 1.0 / 2.0 s: PX4 left offboard 0.57 / 1.05 / 2.0 s after the stream stopped',
                 'single gaps up to 0.9 s tolerated, 1.1 s and longer lost offboard (9 of 9 runs)',
                 'steady 1.5 Hz was safe, 1 Hz was not. tracking error grows as the rate falls (0.47 to 0.59 m/s)',
                 'the timeout runs in PX4 time, so wall-clock thresholds move with simulation speed'],
                foot='simulation only. sweep thresholds are indicative, see FINDINGS.md section 3', size=52)


def previews(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    base = acts_list()
    for style in STYLES:
        set_style(style)
        bg = background()
        acts = {a.key: a for a in base}
        shots = {}
        w = writer(os.path.join(out_dir, f'style_{style}_clip.mp4'))
        t_card = title_card()
        for k in range(int(2.0 * FPS)):
            w.stdin.write(fade(t_card, k, int(2.0 * FPS)).tobytes())
        for key, win in (('A_teleop', (31.0, 39.0, 1.0)), ('C_gap1.5', None)):
            a = acts[key]
            plots, ext = make_plots(a)
            if win is None:
                segs = [segment_times(a.gap_on - 0.8, a.exit_t + 2.4, 0.4)]
            else:
                segs = [segment_times(*win)]
            want = {'A': 34.0} if key == 'A_teleop' else {'C_pause': a.gap_on + 0.9, 'C_fail': a.exit_t + 0.7}
            found = run_segments(a, bg, plots, ext, segs, lambda arr: w.stdin.write(arr.tobytes()), want)
            shots.update(found)
        # stills for B (gap survived) without adding it to the clip
        a = acts['B_gap0.6']
        plots, ext = make_plots(a)
        shots.update(run_segments(a, bg, plots, ext, [segment_times(a.gap_on - 0.2, a.gap_off + 1.6, 1.0)], None,
                                  {'B_gap': a.gap_on + 0.45, 'B_resume': a.gap_off + 0.8}))
        w.stdin.close()
        w.wait()
        for name, arr in shots.items():
            Image.fromarray(arr).save(os.path.join(out_dir, f'style_{style}_{name}.png'))
        Image.fromarray(title_card()).save(os.path.join(out_dir, f'style_{style}_card_title.png'))
        print('style', style, 'done', sorted(shots), flush=True)


def full(style, out_path):
    set_style(style)
    acts = acts_list()
    bg = background()
    tmp = out_path + '.part.mp4'
    w = writer(tmp)
    tc, oc = title_card(), outro_card()
    for k in range(int(4.0 * FPS)):
        w.stdin.write(fade(tc, k, int(4.0 * FPS)).tobytes())
    for a in acts:
        sl = card(a.title, 'every stick motion becomes a velocity setpoint, 20 times per second' if a.kind == 'teleop' else
                  ('PX4 waits 1.0 s before it gives up. does it survive?' if a.gap < 1 else 'longer than the timeout. what does PX4 do?'), size=52)
        for k in range(int(2.2 * FPS)):
            w.stdin.write(fade(sl, k, int(2.2 * FPS)).tobytes())
        plots, ext = make_plots(a)
        segs, cur = [], []
        for t, sp in a.times():
            if cur and sp != cur[-1][1]:
                segs.append(cur)
                cur = []
            cur.append((t, sp))
        segs.append(cur)
        # one continuous reader would be faster, but per-speed segments keep the seek logic simple
        run_segments(a, bg, plots, ext, segs, lambda arr: w.stdin.write(arr.tobytes()))
        print('rendered', a.key, flush=True)
    for k in range(int(10 * FPS)):
        w.stdin.write(fade(oc, k, int(10 * FPS)).tobytes())
    w.stdin.close()
    if w.wait() == 0:
        os.replace(tmp, out_path)
        print('done', out_path)
    else:
        print('FAILED, partial file at', tmp)


if __name__ == '__main__':
    if sys.argv[1] == 'previews':
        previews(sys.argv[2])
    else:
        full(sys.argv[2], sys.argv[3])
