"""Final video composer: two palettes (A black, D paper) of one deliberately plain layout.

Usage: python compose_final.py previews <out_dir>        short clip + stills for A and D
       python compose_final.py full <A|D> <out.mp4>      full video
Data, sync and segment logic come from compose_video.py (Act); this file only decides how it looks.
"""
import os
import subprocess
import sys

import matplotlib
import numpy as np
from matplotlib import font_manager
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
from PIL import Image, ImageDraw, ImageFont

matplotlib.use('Agg')

import compose_video as cv  # noqa: E402
from compose_video import Act, FF, FPS, H, W  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FONTS = os.path.join(HERE, 'assets', 'fonts')
REG, SEMI = os.path.join(FONTS, 'IBMPlexMono-Regular.ttf'), os.path.join(FONTS, 'IBMPlexMono-SemiBold.ttf')

# layout (pixels)
FX, FY, FW, FH = 48, 96, 1296, 729          # Gazebo footage
CX, CW = 1392, 480                          # right column
PY, PH, PW = 844, 212, 584                  # plot row
PXS = [48, 668, 1288]
PAX = (0.13, 0.25, 0.84, 0.64)             # plot axes (left, bottom, width, height) as fractions

PALETTE = {
    'A': dict(bg=(0, 0, 0), fg=(236, 236, 236), muted=(128, 128, 128), accent=(230, 54, 24), on_accent=(0, 0, 0), footage='gray'),
    'D': dict(bg=(240, 240, 236), fg=(18, 18, 18), muted=(120, 120, 116), accent=(230, 54, 24), on_accent=(240, 240, 236), footage='muted'),
}
T = {}


def set_palette(key):
    T.update(PALETTE[key])
    T['key'] = key
    for f in (REG, SEMI):
        font_manager.fontManager.addfont(f)
    matplotlib.rcParams.update({'font.family': 'IBM Plex Mono', 'font.size': 10.5})


def hx(c):
    return '#%02x%02x%02x' % tuple(c)


_f = {}


def font(size, bold=False):
    if (size, bold) not in _f:
        _f[(size, bold)] = ImageFont.truetype(SEMI if bold else REG, size)
    return _f[(size, bold)]


def put(d, xy, s, size=20, bold=False, fill=None):
    d.text(xy, s, font=font(size, bold), fill=fill or T['fg'])


def width(d, s, size, bold=False):
    return d.textlength(s, font=font(size, bold))


def wrap(d, s, size, maxw):
    out, line = [], ''
    for w in s.split():
        trial = (line + ' ' + w).strip()
        if width(d, trial, size) <= maxw:
            line = trial
        else:
            out.append(line)
            line = w
    return out + [line]


# ----------------------------------------------------------------------------- footage


class Feed:
    def __init__(self, path, vstart):
        self.p = subprocess.Popen([FF, '-v', 'error', '-ss', f'{vstart:.3f}', '-i', path, '-vf',
                                   f'fps={FPS},crop=1624:912:32:0,scale={FW}:{FH}:flags=bicubic', '-f', 'rawvideo',
                                   '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE, bufsize=FW * FH * 3 * 6)
        self.idx, self.vstart, self.cur = 0, vstart, None

    def get(self, vt):
        target = int(round((vt - self.vstart) * FPS))
        while self.idx <= target:
            buf = self.p.stdout.read(FW * FH * 3)
            if len(buf) < FW * FH * 3:
                break
            self.cur = np.frombuffer(buf, np.uint8).reshape(FH, FW, 3)
            self.idx += 1
        return self.cur

    def close(self):
        self.p.stdout.close()
        self.p.kill()


def grade(fr):
    lum = fr[..., 0] * 0.299 + fr[..., 1] * 0.587 + fr[..., 2] * 0.114
    if T['footage'] == 'gray':
        g = np.clip((lum - 128) * 1.15 + 128, 0, 255)
        return np.repeat(g[..., None], 3, axis=2).astype(np.uint8)
    mixed = fr.astype(np.float32) * 0.30 + lum[..., None] * 0.70
    return np.clip((mixed - 128) * 1.06 + 128, 0, 255).astype(np.uint8)


def find_drone(fr, prev):
    cx, cy = prev
    x0, x1 = max(0, int(cx) - 180), min(FW, int(cx) + 180)
    y0, y1 = max(0, int(cy) - 130), min(FH, int(cy) + 130)
    c = fr[y0:y1, x0:x1].astype(np.int16)
    m = (c[..., 0] < 80) & (c[..., 1] < 80) & (c[..., 2] < 88)
    if m.sum() > 30:
        ys, xs = np.nonzero(m)
        return (0.6 * cx + 0.4 * (x0 + xs.mean()), 0.6 * cy + 0.4 * (y0 + ys.mean()))
    return prev


# ----------------------------------------------------------------------------- words

TITLES = {'A_teleop': 'Joystick flight', 'B_gap0.6': 'Stream paused for 0.6 s', 'C_gap1.5': 'Stream paused for 1.5 s'}
PHASE = {'warmup': 'Streaming', 'takeoff': 'Takeoff', 'hold': 'Hold', 'turn': 'Yaw turn', 'track': 'Velocity square',
         'settle': 'Settling', 'land': 'Landing', 'idle': 'Hold', 'forward': 'Forward', 'right': 'Sideways',
         'climb': 'Climb', 'descend': 'Descend', 'carve': 'Carving turn'}


def words(act, t):
    """(title, body, level) level 0 normal, 1 attention, 2 alert."""
    i = act.idx(t)
    st = act.stage[i]
    if act.kind == 'teleop':
        if t < act.t_arm + 0.3:
            return ('Streaming first', 'The node sends setpoints at 20 Hz before it asks PX4 for offboard mode. Without that stream PX4 refuses the switch.', 0)
        if st == 'idle' and t < 13.5:
            return ('Takeoff', 'Arm, switch to offboard, climb to 5 m on a position setpoint.', 0)
        tab = {
            'idle': ('Sticks centred', 'Zero velocity setpoint. The stream keeps running, so PX4 stays in offboard mode.', 0),
            'forward': ('Right stick forward', 'Deadzone, 2 m/s limit, then body frame to NED using the heading. The drone faces east, so forward is east.', 0),
            'right': ('Right stick sideways', 'To the right of an east-facing drone is south. I had this sign wrong at first; this overlay made it obvious.', 1),
            'climb': ('Left stick up', 'Climb, limited to 1 m/s. In NED a climb is a negative down velocity.', 0),
            'descend': ('Left stick down', 'Same limit, other direction.', 0),
            'carve': ('Forward and yaw together', 'Both sticks at once. The body to NED conversion follows the heading as it changes.', 0),
            'land': ('Button A', 'Land command. PX4 lands and disarms; the stream is no longer needed.', 0),
        }
        return tab.get(st, tab['idle'])
    if st == 'warmup':
        return ('Streaming first', 'Setpoints go out at 20 Hz before the script asks PX4 for offboard mode.', 0)
    if st == 'takeoff':
        return ('Takeoff', 'Climb to 5 m, with setpoints streaming the whole time.', 0)
    if st == 'hold':
        return ('Hold', 'Nothing changes, but the heartbeat keeps going.', 0)
    if st == 'turn':
        return ('Yaw turn', '90 degrees, still 20 Hz.', 0)
    g = f'{act.gap:g} s'
    if act.exit_t and act.exit_t <= t < act.exit_t + 4.5:
        return ('Timeout', 'After 1.0 s without a setpoint PX4 assumes the companion computer is gone and returns to launch.', 2)
    if st == 'track':
        gap_end = act.gap_off if act.gap_off else float('inf')
        if act.gap_on and t < act.gap_on:
            return ('Velocity square', 'A setpoint every 50 ms. PX4 only complains after 1.0 s of silence.', 0)
        if t < gap_end and not (act.exit_t and t >= act.exit_t):
            return ('Stream paused', 'No message arrives. PX4 counts the silence against its 1.0 s timeout.', 1)
        return ('Stream is back', f'{g} of silence is under the 1.0 s timeout, so no failsafe. Only the command that changed during the gap was missed.', 0)
    if st == 'settle':
        return ('Square done', '', 0)
    if act.exit_t:
        return ('Landing', 'PX4 is out of offboard, so the script just sends land. End of the test.', 0)
    return ('Landing', 'The script sends land and stops streaming. In land mode that is harmless.', 0)


# ----------------------------------------------------------------------------- plots


def build_plots(act):
    """Per plot: a static image (frame, ticks, labels, fixed lines) and an RGBA data layer revealed up to the cursor."""
    ts = act.times()
    t0, t1 = ts[0][0], ts[-1][0]
    m = (act.t >= t0) & (act.t <= t1)
    t = act.t[m]
    fg, mu, ac, bg = hx(T['fg']), hx(T['muted']), hx(T['accent']), hx(T['bg'])
    size = (PW / 100, PH / 100)
    out = []

    def static_fig(ylab, ylim, raster=False):
        fig = Figure(figsize=size, dpi=100, facecolor=bg)
        ax = fig.add_axes([PAX[0], PAX[1], PAX[2], 0.50 if raster else PAX[3]] if raster else list(PAX))
        ax.set_facecolor(bg)
        for k in ('top', 'right'):
            ax.spines[k].set_visible(False)
        for k in ('left', 'bottom'):
            ax.spines[k].set_color(fg)
            ax.spines[k].set_linewidth(0.8)
        ax.tick_params(colors=fg, length=4, width=0.9, labelsize=10)
        ax.set_xlim(t0, t1)
        ax.set_ylim(*ylim)
        ax.set_ylabel(ylab, color=fg, fontsize=11)
        ax.set_xlabel('time (s)', color=fg, fontsize=11)
        return fig, ax

    def data_fig(ylim, raster=False):
        fig = Figure(figsize=size, dpi=100, facecolor=(0, 0, 0, 0))
        ax = fig.add_axes([PAX[0], PAX[1], PAX[2], 0.50 if raster else PAX[3]] if raster else list(PAX))
        ax.patch.set_alpha(0)
        ax.set_axis_off()
        ax.set_xlim(t0, t1)
        ax.set_ylim(*ylim)
        return fig, ax

    def failsafe(ax):
        if act.exit_t:
            ax.axvspan(act.exit_t, t1, color=ac, alpha=0.13, lw=0)
            ax.axvline(act.exit_t, color=ac, lw=1.4)

    def grab(fig, rgba=False):
        canvas = FigureCanvasAgg(fig)
        canvas.draw()
        a = np.asarray(canvas.buffer_rgba()).copy()
        return a if rgba else a[..., :3]

    # speed
    yl = (0, 2.1 if act.kind == 'teleop' else 1.25)
    fs, ax = static_fig('speed (m/s)', yl)
    ax.plot([], [], color=fg, lw=1.8, label='measured')
    ax.plot([], [], color=fg, lw=1.0, ls=(0, (4, 3)), label='command')
    ax.legend(loc='upper left', frameon=False, fontsize=10, labelcolor=fg, handlelength=2.2)
    fd, dx = data_fig(yl)
    failsafe(dx)
    dx.plot(t, act.cmd_h[m], color=fg, lw=1.0, ls=(0, (4, 3)))
    dx.plot(t, act.sph[m], color=fg, lw=1.8)
    out.append((grab(fs), grab(fd, True)))
    # altitude
    yl = (-0.2, 6.6)
    fs, ax = static_fig('altitude (m)', yl)
    fd, dx = data_fig(yl)
    failsafe(dx)
    dx.plot(t, act.alt[m], color=fg, lw=1.8)
    out.append((grab(fs), grab(fd, True)))
    # stream
    yl = (0, 1.5)
    fs, ax = static_fig('silence (s)', yl, raster=True)
    ax.axhline(1.0, color=ac, lw=1.0, ls=(0, (4, 3)))
    ax.text(t0 + 0.01 * (t1 - t0), 1.05, 'PX4 timeout 1.0 s', color=ac, fontsize=10)
    rx = fs.add_axes([PAX[0], 0.80, PAX[2], 0.12])
    rx.set_axis_off()
    rx.set_xlim(t0, t1)
    rx.text(0.0, 1.12, 'messages', transform=rx.transAxes, fontsize=10, color=fg, va='bottom')
    fd, dx = data_fig(yl, raster=True)
    failsafe(dx)
    dx.plot(t, np.minimum(act.since[m], 1.5), color=fg, lw=1.6)
    rd = fd.add_axes([PAX[0], 0.80, PAX[2], 0.12])
    rd.set_axis_off()
    rd.patch.set_alpha(0)
    rd.set_xlim(t0, t1)
    rd.set_ylim(0, 1)
    sent = act.sent_t[(act.sent_t >= t0) & (act.sent_t <= t1)]
    rd.vlines(sent, 0, 1, color=fg, lw=0.5)
    out.append((grab(fs), grab(fd, True)))
    return out, (t0, t1)


def draw_plots(arr, plots, extents, t):
    t0, t1 = extents
    for x, (static, data) in zip(PXS, plots):
        axl, axr = int(PAX[0] * PW), int((PAX[0] + PAX[2]) * PW)
        cx = int(axl + (min(max(t, t0), t1) - t0) / (t1 - t0) * (axr - axl))
        img = static.copy()
        a = data[:, axl:cx, 3:4].astype(np.float32) / 255.0
        img[:, axl:cx] = (img[:, axl:cx] * (1 - a) + data[:, axl:cx, :3] * a).astype(np.uint8)
        arr[PY:PY + PH, x:x + PW] = img


# ----------------------------------------------------------------------------- drawing


def title_bar(d, act, t, speed, nav):
    put(d, (48, 30), TITLES[act.key], 30, True)
    if T['key'] == 'D':
        d.line((48, 86, W - 48, 86), fill=T['fg'], width=2)
    mode = {14: 'offboard', 5: 'failsafe: return to launch', 18: 'landing', 4: 'hold'}.get(int(nav), 'PX4')
    col = T['accent'] if int(nav) == 5 else T['fg']
    right = W - 48
    put(d, (right - width(d, mode, 22, True), 28), mode, 22, True, col)
    info = f't = {t:5.1f} s' + (f'    {speed:g}x' if speed != 1 else '')
    put(d, (right - width(d, info, 16), 62), info, 16, False, T['muted'])


def footage_marks(d, act, t, i, ring, tt):
    x, y = FX + ring[0], FY + ring[1]
    r = 44
    d.ellipse((x - r, y - r, x + r, y + r), outline=T['accent'], width=3)
    d.line((x + r * 0.72, y - r * 0.72, x + r + 26, y - r - 20), fill=T['fg'], width=1)
    label = [('x500', True), (f'{max(act.alt[i], 0):.1f} m   {act.sph[i]:.1f} m/s', False)]
    bx, by = x + r + 26, y - r - 52
    wmax = max(width(d, label[0][0], 18, True), width(d, label[1][0], 16)) + 20
    d.rectangle((bx, by, bx + wmax, by + 56), fill=T['bg'])
    put(d, (bx + 10, by + 4), label[0][0], 18, True)
    put(d, (bx + 10, by + 30), label[1][0], 16)
    tag = 'Failsafe return' if (act.exit_t and act.exit_t <= t < act.exit_t + 4.5) else PHASE.get(act.stage[i], '')
    d.rectangle((FX, FY, FX + width(d, tag, 20, True) + 28, FY + 38), fill=T['bg'])
    put(d, (FX + 14, FY + 6), tag, 20, True)
    if act.kind != 'gap' or not act.gap_on or act.gap_on > t:
        return
    silent = act.since[i]
    in_gap = (act.gap_off is None or t < act.gap_off) and not (act.exit_t and t >= act.exit_t)
    cx = FX + FW // 2
    if in_gap:
        s = 'Stream paused'
        hot = silent >= 0.7
        w = width(d, s, 24, True) + 40
        d.rectangle((cx - w / 2, FY, cx + w / 2, FY + 46), fill=T['accent'] if hot else T['bg'])
        put(d, (cx - w / 2 + 20, FY + 8), s, 24, True, T['on_accent'] if hot else T['fg'])
    elif act.exit_t and act.exit_t <= t < act.exit_t + 4.5:
        d.rectangle((FX, FY, FX + FW - 1, FY + FH - 1), outline=T['accent'], width=6)
        s = 'Offboard lost. Return to launch.'
        w = width(d, s, 26, True) + 40
        d.rectangle((cx - w / 2, FY, cx + w / 2, FY + 52), fill=T['accent'])
        put(d, (cx - w / 2 + 20, FY + 10), s, 26, True, T['on_accent'])
    elif act.gap_off and not act.exit_t and act.gap_off <= t < act.gap_off + 4.0:
        s = 'Stream resumed. Offboard kept.'
        w = width(d, s, 24, True) + 40
        d.rectangle((cx - w / 2, FY, cx + w / 2, FY + 46), fill=T['fg'])
        put(d, (cx - w / 2 + 20, FY + 8), s, 24, True, T['bg'])


def sticks(d, act, i):
    df = act.df
    yaw, up, right, fwd = (float(df.ax_yaw[i]), float(df.ax_up[i]), float(df.ax_right[i]), float(df.ax_fwd[i]))
    y0 = FY + 20
    for cx, ah, av, name, sub in ((CX + 110, yaw, up, 'left stick', 'yaw, climb'), (CX + CW - 110, right, fwd, 'right stick', 'sideways, forward')):
        cy = y0 + 84
        on = abs(ah) > 0.05 or abs(av) > 0.05
        d.ellipse((cx - 72, cy - 72, cx + 72, cy + 72), outline=T['fg'] if on else T['muted'], width=2)
        nx, ny = cx - ah * 44, cy - av * 44
        d.ellipse((nx - 15, ny - 15, nx + 15, ny + 15), fill=T['fg'] if on else None, outline=T['fg'] if on else T['muted'], width=2)
        put(d, (cx - width(d, name, 17, True) / 2, cy + 84), name, 17, True, T['fg'] if on else T['muted'])
        put(d, (cx - width(d, sub, 14) / 2, cy + 108), sub, 14, False, T['muted'])
    pressed = int(df.btn_land[i]) == 1
    bx, by = CX + CW // 2, y0 + 30
    d.ellipse((bx - 18, by - 18, bx + 18, by + 18), fill=T['fg'] if pressed else None, outline=T['fg'] if pressed else T['muted'], width=2)
    put(d, (bx - 6, by - 13), 'A', 20, True, T['bg'] if pressed else T['muted'])
    put(d, (bx - width(d, 'land', 14) / 2, by + 24), 'land', 14, pressed, T['fg'] if pressed else T['muted'])
    n, e, dn = act.ned[i]
    yy = FY + 330
    put(d, (CX, yy), 'setpoint sent to PX4', 15, False, T['muted'])
    put(d, (CX, yy + 24), f'north {n:+5.2f}   east {e:+5.2f}', 19)
    put(d, (CX, yy + 50), f'down  {dn:+5.2f}   yaw  {act.yaw_rate[i]:+5.2f}', 19)
    put(d, (CX, yy + 82), 'm/s and rad/s', 14, False, T['muted'])


def silence(d, act, i, t):
    silent = float(act.since[i])
    lost = act.exit_t is not None and t >= act.exit_t
    landing = act.stage[i] == 'land' and not lost
    hot = lost or (silent >= 0.7 and not landing)
    put(d, (CX, FY + 8), 'time since the last setpoint', 16, False, T['muted'])
    shown = '--' if (lost or landing) else f'{silent:4.2f} s'
    put(d, (CX, FY + 36), shown, 100, True, T['accent'] if hot else (T['muted'] if landing else T['fg']))
    by = FY + 190
    d.line((CX, by, CX + CW, by), fill=T['muted'], width=2)
    frac = 0 if landing else (1.0 if lost else min(1.0, silent / 1.2))
    if frac > 0:
        d.rectangle((CX, by - 4, CX + CW * frac, by + 4), fill=T['accent'] if hot else T['fg'])
    tx = CX + CW * (1.0 / 1.2)
    d.line((tx, by - 16, tx, by + 16), fill=T['accent'], width=3)
    put(d, (tx - width(d, 'timeout 1.0 s', 15, True) + 8, by + 24), 'timeout 1.0 s', 15, True, T['accent'])


def caption(d, w3):
    title, body, level = w3
    y = FY + FH - 250
    put(d, (CX, y), title, 26, True, T['accent'] if level == 2 else T['fg'])
    for k, line in enumerate(wrap(d, body, 18, CW)):
        put(d, (CX, y + 42 + k * 27), line, 18)


def card(title, sub='', lines=(), size=72):
    img = Image.new('RGB', (W, H), T['bg'])
    d = ImageDraw.Draw(img)
    y = 380 if not lines else 250
    put(d, (96, y), title, size, True)
    if sub:
        for k, line in enumerate(wrap(d, sub, 30, W - 260)):
            put(d, (98, y + size + 36 + k * 42), line, 30, False, T['muted'])
    yy = y + size + 110
    for s in lines:
        for k, line in enumerate(wrap(d, s, 26, W - 300)):
            put(d, (98, yy + k * 36), line, 26)
        yy += 36 * len(wrap(d, s, 26, W - 300)) + 22
    return np.array(img)


def fade(arr, k, n):
    a = min(1.0, k / (0.3 * FPS), (n - k) / (0.3 * FPS))
    return (arr.astype(np.float32) * a + np.array(T['bg']) * (1 - a)).astype(np.uint8)


# ----------------------------------------------------------------------------- render


def one_frame(act, base, plots, ext, feed, t, speed, ring, tt):
    fr = feed.get(t + act.off)
    if fr is None:
        return None, ring
    i = act.idx(t)
    ring = find_drone(fr, ring)
    arr = base.copy()
    arr[FY:FY + FH, FX:FX + FW] = grade(fr)
    draw_plots(arr, plots, ext, t)
    img = Image.fromarray(arr)
    d = ImageDraw.Draw(img)
    title_bar(d, act, t, speed, act.nav[i])
    footage_marks(d, act, t, i, ring, tt)
    (sticks(d, act, i) if act.kind == 'teleop' else silence(d, act, i, t))
    caption(d, words(act, t))
    return np.array(img), ring


def run(act, base, plots, ext, segs, sink, want=None):
    found = {}
    for times in segs:
        feed = Feed(os.path.join(cv.RAW, f'act_{act.key}.mp4'), max(0.0, times[0][0] + act.off))
        ring = (FW * 0.5, FH * 0.46)
        n = len(times)
        for k, (t, sp) in enumerate(times):
            out, ring = one_frame(act, base, plots, ext, feed, t, sp, ring, k / FPS)
            if out is None:
                break
            for name, ts in (want or {}).items():
                if name not in found and abs(t - ts) <= sp / FPS:
                    found[name] = out
            if sink:
                sink(fade(out, k, n) if (k < 8 or n - k < 8) else out)
        feed.close()
    return found


def seg(a, b, speed):
    out, t = [], a
    while t < b:
        out.append((t, speed))
        t += speed / FPS
    return out


def acts():
    L = [Act('A_teleop', 'teleop', '', ''), Act('B_gap0.6', 'gap', '', '', gap=0.6), Act('C_gap1.5', 'gap', '', '', gap=1.5)]
    for a in L:
        a.detect_offset()
    return L


def encoder(path):
    return subprocess.Popen([FF, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                             '-preset', 'fast', '-threads', '6', '-crf', '18', '-pix_fmt', 'yuv420p', path], stdin=subprocess.PIPE)


def base_frame():
    return np.array(Image.new('RGB', (W, H), T['bg']))


def title_card():
    return card('PX4 offboard control', 'A ROS 2 node flies a simulated quadcopter. What happens when its command stream goes quiet?')


def slate(a):
    sub = {'A_teleop': 'Every stick motion becomes a velocity setpoint, 20 times per second.',
           'B_gap0.6': 'PX4 waits 1.0 s before it gives up. Does it survive?',
           'C_gap1.5': 'Longer than the timeout. What does PX4 do?'}[a.key]
    return card({'A_teleop': '1  Joystick flight', 'B_gap0.6': '2  Stream paused for 0.6 s', 'C_gap1.5': '3  Stream paused for 1.5 s'}[a.key], sub, size=56)


def outro_card():
    return card('Measurements', '', [
        'With the timeout set to 0.5, 1.0 and 2.0 s, PX4 left offboard 0.57, 1.05 and 2.0 s after the stream stopped.',
        'Single gaps up to 0.9 s were tolerated. Gaps of 1.1 s and longer lost offboard mode in 9 of 9 runs.',
        'A steady 1.5 Hz stream was safe and 1 Hz was not. Tracking error grows as the rate falls, from 0.47 to 0.59 m/s.',
        'The timeout runs in PX4 time, so a threshold measured in wall-clock Hz moves with simulation speed.',
        'Simulation only: PX4 SITL with Gazebo, 2 to 3 runs per condition. Caveats are in FINDINGS.md.'], size=52)


def previews(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    L = acts()
    for key in ('A', 'D'):
        set_palette(key)
        base = base_frame()
        shots = {}
        w = encoder(os.path.join(out_dir, f'final_{key}_clip.mp4'))
        tc = title_card()
        for k in range(int(2.0 * FPS)):
            w.stdin.write(fade(tc, k, int(2.0 * FPS)).tobytes())
        sink = lambda arr: w.stdin.write(arr.tobytes())  # noqa: E731
        a = L[0]
        plots, ext = build_plots(a)
        shots.update(run(a, base, plots, ext, [seg(31.0, 39.0, 1.0)], sink, {'A_carve': 34.0}))
        shots.update(run(a, base, plots, ext, [seg(15.5, 17.5, 1.0)], None, {'A_forward': 16.6}))
        b = L[1]
        plots, ext = build_plots(b)
        shots.update(run(b, base, plots, ext, [seg(b.gap_on - 0.2, b.gap_off + 1.6, 1.0)], None, {'B_pause': b.gap_on + 0.4, 'B_resume': b.gap_off + 0.8}))
        c = L[2]
        plots, ext = build_plots(c)
        shots.update(run(c, base, plots, ext, [seg(c.gap_on - 0.8, c.exit_t + 2.4, 0.4)], sink, {'C_pause': c.gap_on + 0.95, 'C_fail': c.exit_t + 0.7}))
        w.stdin.close()
        w.wait()
        for name, arr in shots.items():
            Image.fromarray(arr).save(os.path.join(out_dir, f'final_{key}_{name}.png'))
        Image.fromarray(tc).save(os.path.join(out_dir, f'final_{key}_title.png'))
        Image.fromarray(outro_card()).save(os.path.join(out_dir, f'final_{key}_outro.png'))
        print('style', key, 'done', sorted(shots), flush=True)


def full(key, out_path):
    set_palette(key)
    L = acts()
    base = base_frame()
    tmp = out_path + '.part.mp4'
    w = encoder(tmp)
    sink = lambda arr: w.stdin.write(arr.tobytes())  # noqa: E731
    tc = title_card()
    for k in range(int(3.5 * FPS)):
        w.stdin.write(fade(tc, k, int(3.5 * FPS)).tobytes())
    for a in L:
        sl = slate(a)
        for k in range(int(2.2 * FPS)):
            w.stdin.write(fade(sl, k, int(2.2 * FPS)).tobytes())
        plots, ext = build_plots(a)
        segs, cur = [], []
        for t, sp in a.times():
            if cur and sp != cur[-1][1]:
                segs.append(cur)
                cur = []
            cur.append((t, sp))
        segs.append(cur)
        run(a, base, plots, ext, segs, sink)
        print('rendered', a.key, flush=True)
    oc = outro_card()
    for k in range(int(12 * FPS)):
        w.stdin.write(fade(oc, k, int(12 * FPS)).tobytes())
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
