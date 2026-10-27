"""Cinematic cut: 2.39:1 letterbox, restrained grade, slow motion at the key moments, captions in the bars,
a heartbeat dot and a silence counter, and a sound track made from the message stream itself.

Usage: python compose_cinema.py stills <out_dir>     one frame per shot, to judge the look
       python compose_cinema.py full <out.mp4>        the finished film
Sources: results/cinematic_raw/take_A (+ take_C if present), results/showcase_raw acts. Sync is found from the footage.
"""
import math
import os
import subprocess
import sys
import wave

import numpy as np
import pandas as pd
from PIL import Image, ImageDraw, ImageFont

import compose_video as cv
from compose_video import FF

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
CINE = os.path.join(ROOT, 'results', 'cinematic_raw')
REG = os.path.join(HERE, 'assets', 'fonts', 'IBMPlexMono-Regular.ttf')
SEMI = os.path.join(HERE, 'assets', 'fonts', 'IBMPlexMono-SemiBold.ttf')
W, H, FPS = 1920, 1080, 30
FH_ = 804                      # footage height inside the letterbox (2.39:1)
FY = (H - FH_) // 2
BG, FG, MUTED, ACCENT = (0, 0, 0), (236, 236, 236), (128, 128, 128), (230, 54, 24)
SR = 48000

_f = {}


def font(size, bold=False):
    if (size, bold) not in _f:
        _f[(size, bold)] = ImageFont.truetype(SEMI if bold else REG, size)
    return _f[(size, bold)]


# ------------------------------------------------------------------------------------------ sources


class Take:
    """Flight log plus video, with the fields the edit needs. Works for the driver log (teleop) and the study_node log."""

    def __init__(self, key, video, csv, sync_window=None, cut_at=None, kind='gap'):
        df = pd.read_csv(csv)
        for c in ['x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'nav_state']:
            df[c] = pd.to_numeric(df[c], errors='coerce')
        df = df.ffill().bfill().reset_index(drop=True)
        self.key, self.video, self.kind = key, video, kind
        self.t = df.t.to_numpy()
        self.alt = -df.z.to_numpy()
        self.sph = np.hypot(df.vx, df.vy).to_numpy()
        self.nav = df.nav_state.to_numpy()
        if 'sent' in df:
            self.sent_t = self.t[df.sent.to_numpy() == 1]
        else:   # joystick node streams on a 20 Hz timer
            self.sent_t = np.arange(self.t[0], self.t[-1], 0.05)
        idx = np.searchsorted(self.sent_t, self.t, side='right') - 1
        self.since = self.t - self.sent_t[np.clip(idx, 0, len(self.sent_t) - 1)]
        ex = np.nonzero((self.nav[1:] != 14) & (self.nav[:-1] == 14) & (self.t[1:] > 3))[0]
        self.exit_t = self.t[ex[0] + 1] if len(ex) and self.nav[ex[0] + 1] == 5 else None
        self.gap_on = self.gap_off = None
        if kind == 'gap':
            g = np.diff(self.sent_t)
            if self.exit_t:
                self.gap_on = float(self.sent_t[self.sent_t <= self.exit_t][-1])
            elif len(g) and g.max() > 0.4:
                k = int(g.argmax())
                self.gap_on, self.gap_off = float(self.sent_t[k]), float(self.sent_t[k + 1])
        self.off = None

    def idx(self, t):
        return int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))

    def find_offset(self, lo, hi, expected):
        """Video time of the biggest scene change inside [lo, hi] minus the log time it was scripted at."""
        raw = subprocess.run([FF, '-v', 'error', '-ss', str(lo), '-t', str(hi - lo), '-i', self.video, '-vf',
                              'fps=20,scale=96:54,format=gray', '-f', 'rawvideo', '-'], capture_output=True).stdout
        fr = np.frombuffer(raw, np.uint8).reshape(-1, 54, 96).astype(np.float32)
        d = np.abs(np.diff(fr, axis=0)).mean(axis=(1, 2))
        first = int(np.argmax(d > 0.5 * d.max()))
        self.off = round(lo + (first + 1) / 20.0 - expected, 2)
        print(f'{self.key}: offset {self.off} s (spike {d.max():.1f})', flush=True)


def act_source(key, gap):
    a = cv.Act(key, 'gap', '', '', gap=gap)
    a.detect_offset()
    t = Take.__new__(Take)
    t.key, t.video, t.kind = key, os.path.join(cv.RAW, f'act_{key}.mp4'), 'gap'
    t.t, t.alt, t.sph, t.nav, t.sent_t, t.since = a.t, a.alt, a.sph, a.nav, a.sent_t, a.since
    t.exit_t, t.gap_on, t.gap_off, t.off, t.idx = a.exit_t, a.gap_on, a.gap_off, a.off, a.idx
    return t


# ------------------------------------------------------------------------------------------ footage


GRADE = ('crop=1624:912:32:0,crop=1624:680:0:{y0},scale=1920:{h}:flags=lanczos,'
         'eq=contrast=1.12:saturation=0.60:gamma=0.93:brightness=-0.02,'
         'colorbalance=rs=0.02:bs=-0.02:rh=0.04:bh=-0.03,noise=alls=7:allf=t,vignette=PI/5')


class Reader:
    def __init__(self, path, vstart, y0):
        vf = f'fps={FPS},' + GRADE.format(y0=y0, h=FH_)
        self.p = subprocess.Popen([FF, '-v', 'error', '-ss', f'{max(0.0, vstart):.3f}', '-i', path, '-vf', vf, '-f', 'rawvideo',
                                   '-pix_fmt', 'rgb24', '-'], stdout=subprocess.PIPE, bufsize=1920 * FH_ * 3 * 4)
        self.idx, self.vstart, self.cur = 0, max(0.0, vstart), None

    def get(self, vt):
        target = int(round((vt - self.vstart) * FPS))
        while self.idx <= target:
            buf = self.p.stdout.read(1920 * FH_ * 3)
            if len(buf) < 1920 * FH_ * 3:
                break
            self.cur = np.frombuffer(buf, np.uint8).reshape(FH_, 1920, 3)
            self.idx += 1
        return self.cur

    def close(self):
        self.p.stdout.close()
        self.p.kill()


def push(frame, z):
    if z <= 1.0005:
        return frame
    h, w = frame.shape[:2]
    ch, cw = int(h / z), int(w / z)
    y0, x0 = (h - ch) // 2, (w - cw) // 2
    im = Image.fromarray(frame[y0:y0 + ch, x0:x0 + cw]).resize((w, h), Image.BILINEAR)
    return np.asarray(im)


# ------------------------------------------------------------------------------------------ drawing


def text_w(d, s, size, bold=False):
    return d.textlength(s, font=font(size, bold))


def centered(d, y, s, size, fill, bold=False):
    d.text(((W - text_w(d, s, size, bold)) / 2, y), s, font=font(size, bold), fill=fill)


def fade_alpha(t, a, b, fi=0.35, fo=0.35):
    return max(0.0, min(1.0, (t - a) / fi, (b - t) / fo))


def blend(c, a):
    return tuple(int(BG[i] + (c[i] - BG[i]) * a) for i in range(3))


def overlay(canvas, shot, t, i, tick_recent):
    img = Image.fromarray(canvas)
    d = ImageDraw.Draw(img)
    src = shot['src']
    y_bar = FY + FH_ + 24
    # caption in the bottom bar
    for a, b, s, hot in shot.get('captions', []):
        al = fade_alpha(t, a, b)
        if al > 0:
            col = blend(ACCENT if hot else FG, al)
            centered(d, y_bar + 18, s, 26, col)
    if shot.get('counter'):
        since = float(src.since[i])
        lost = src.exit_t is not None and t >= src.exit_t
        hot = lost or since >= 0.7
        col = ACCENT if hot else FG
        d.text((96, y_bar + 6), 'silence since the last setpoint', font=font(15), fill=MUTED)
        d.text((96, y_bar + 26), '--' if lost else f'{since:4.2f} s', font=font(46, True), fill=col)
        r, cx, cy = 9, 1824, y_bar + 52
        if lost:
            d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=ACCENT, width=2)
        elif tick_recent:
            d.ellipse((cx - r, cy - r, cx + r, cy + r), fill=FG)
        else:
            d.ellipse((cx - r, cy - r, cx + r, cy + r), outline=MUTED, width=2)
        d.text((cx - 110 - text_w(d, 'messages', 15), cy - 10), 'messages', font=font(15), fill=MUTED)
        if src.exit_t is not None and src.exit_t <= t < src.exit_t + 0.9:
            k = 1 - (t - src.exit_t) / 0.9
            for w_ in range(1, 9):
                c = tuple(int(BG[j] + (ACCENT[j] - BG[j]) * k * (1 - w_ / 9)) for j in range(3))
                d.rectangle((w_ * 2, FY + w_ * 2, W - w_ * 2, FY + FH_ - w_ * 2), outline=c, width=2)
    return np.asarray(img)


def card_frame(lines, k, n, typed=None):
    img = Image.new('RGB', (W, H), BG)
    d = ImageDraw.Draw(img)
    for row in lines:
        s, size, col, y, t_in = row
        al = max(0.0, min(1.0, (k / FPS - t_in) / 0.5, (n - k) / (0.5 * FPS)))
        if typed and row is lines[0]:
            nchar = int(max(0, (k / FPS - t_in)) / 0.075)
            s = s[:nchar] + ('_' if (k // 12) % 2 == 0 and nchar < len(s) + 6 else '')
        d.text((120, y), s, font=font(size, size > 40), fill=blend(col, al))
    return np.asarray(img)


# ------------------------------------------------------------------------------------------ shots


def build_shots():
    A = Take('A', os.path.join(CINE, 'take_A.mp4'), os.path.join(CINE, 'take_A.csv'), kind='teleop')
    A.find_offset(12, 24, 11.0)          # scripted cut from static to follow camera at t = 11 s
    B = act_source('B_gap0.6', 0.6)
    have_c = os.path.exists(os.path.join(CINE, 'take_C.mp4'))
    if have_c:
        C = Take('C', os.path.join(CINE, 'take_C.mp4'), os.path.join(CINE, 'take_C.csv'), kind='gap')
        C.find_offset(2, 14, 0.0)        # director pulses the camera at t = 0
    else:
        C = act_source('C_gap1.5', 1.5)
    shots = []
    shots.append(dict(src=A, a=2.0, b=10.6, speed=0.8, y0=70, push=(1.0, 1.05), sound='A',
                      captions=[(3.0, 7.0, 'A ROS 2 node sends setpoints to PX4, twenty a second', False),
                                (8.4, 10.3, 'PX4 follows it. Offboard mode.', False)]))
    shots.append(dict(src=A, a=11.9, b=23.0, speed=1.0, y0=60, push=(1.0, 1.0), sound='A',
                      captions=[(12.6, 17.6, 'Right stick forward, then sideways', False),
                                (19.4, 22.8, 'Each stick value becomes a velocity setpoint', False)]))
    shots.append(dict(src=A, a=25.6, b=36.6, speed=1.0, y0=60, push=(1.0, 1.0), sound='A',
                      captions=[(26.2, 35.8, 'Forward and yaw together. The conversion follows the heading.', False)]))
    shots.append(dict(src=A, a=41.0, b=47.0, speed=1.25, y0=30, push=(1.0, 1.0), sound='A', captions=[]))
    g = B.gap_on
    shots.append(dict(src=B, a=g - 1.6, b=B.gap_off + 2.2, speed=0.55, y0=70, push=(1.0, 1.0), counter=True, sound='B',
                      captions=[(g - 1.4, g + 0.8, 'Pause the stream for 0.6 s', False),
                                (B.gap_off + 0.1, B.gap_off + 2.0, 'PX4 waits. Offboard kept.', False)]))
    gc, ex = C.gap_on, C.exit_t
    shots.append(dict(src=C, a=gc - 5.0, b=gc - 1.3, speed=1.0, y0=70, push=(1.0, 1.03), counter=True, sound='C',
                      captions=[(gc - 4.8, gc - 1.5, 'Now pause it for 1.5 s', False)]))
    shots.append(dict(src=C, a=gc - 1.3, b=ex + 2.4, speed=0.5, y0=70, push=(1.0, 1.0), counter=True, sound='C',
                      captions=[(ex + 0.1, ex + 2.3, 'Offboard lost. Return to launch.', True)]))
    shots.append(dict(src=C, a=ex + 2.4, b=ex + 8.0, speed=1.0, y0=70, push=(1.0, 1.0), counter=True, sound='C',
                      captions=[(ex + 2.6, ex + 7.6, 'After 1.0 s of silence PX4 takes the drone back', True)]))
    return shots, have_c


def shot_times(shot):
    out, t = [], shot['a']
    while t < shot['b']:
        out.append(t)
        t += shot['speed'] / FPS
    return out


def render_shot(shot, sink, events, base_frame):
    src = shot['src']
    times = shot_times(shot)
    reader = Reader(src.video, shot['a'] + src.off, shot['y0'])
    n = len(times)
    last_tick_idx = -1
    first_out = events['frames']
    for k, t in enumerate(times):
        fr = reader.get(t + src.off)
        if fr is None:
            break
        z = shot['push'][0] + (shot['push'][1] - shot['push'][0]) * k / max(1, n - 1)
        fr = push(fr, z)
        canvas = np.zeros((H, W, 3), np.uint8)
        canvas[FY:FY + FH_] = fr
        i = src.idx(t)
        j = np.searchsorted(src.sent_t, t, side='right') - 1
        tick_recent = j >= 0 and (t - src.sent_t[j]) < 0.06 and j % 5 == 0
        out = overlay(canvas, shot, t, i, tick_recent)
        a = min(1.0, k / 8.0, (n - 1 - k) / 8.0) if shot.get('dip') else 1.0
        if a < 1.0:
            out = (out.astype(np.float32) * a).astype(np.uint8)
        sink(out)
        # audio events, in output time
        ot = (first_out + k) / FPS
        if j >= 0 and j != last_tick_idx and (t - src.sent_t[j]) < shot['speed'] / FPS + 0.001 and j % 5 == 0:
            events['ticks'].append(ot)
        last_tick_idx = j
        events['air'].append((ot, float(np.clip(src.alt[i] / 3.0, 0, 1)), float(np.clip(src.sph[i] / 2.0, 0, 1))))
        if src.exit_t is not None and shot['sound'] == 'C':
            if abs(t - src.exit_t) <= shot['speed'] / FPS / 2 + 1e-6:
                events['hit'] = ot
            if src.gap_on is not None and src.gap_on <= t < src.exit_t:
                events['riser'] = (events['riser'][0] if events['riser'] else ot, ot)
    events['frames'] += k + 1
    reader.close()


# ------------------------------------------------------------------------------------------ sound


def band_noise(n, lo, hi, rng):
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / SR)
    spec *= ((f >= lo) & (f <= hi)).astype(float)
    x = np.fft.irfft(spec, n)
    return x / (np.abs(x).max() + 1e-9)


def synth(events, total_s, path):
    rng = np.random.default_rng(3)
    n = int(total_s * SR)
    t = np.arange(n) / SR
    bed = 0.050 * np.sin(2 * np.pi * 55 * t) + 0.032 * np.sin(2 * np.pi * 82.4 * t + 0.5) + 0.020 * np.sin(2 * np.pi * 110 * t)
    bed *= 0.75 + 0.25 * np.sin(2 * np.pi * 0.07 * t)
    bed += 0.012 * band_noise(n, 80, 500, rng)
    # rotor: band noise whose level follows altitude and speed in the film
    if events['air']:
        ot = np.array([a[0] for a in events['air']])
        alt = np.array([a[1] for a in events['air']])
        spd = np.array([a[2] for a in events['air']])
        env = np.interp(t, ot, 0.04 + 0.16 * alt + 0.10 * spd * alt, left=0, right=0)
        bed += 0.11 * env * band_noise(n, 180, 1400, rng)
    out = bed.copy()
    click = np.exp(-np.arange(int(0.012 * SR)) / (0.0025 * SR)) * np.sin(2 * np.pi * 1800 * np.arange(int(0.012 * SR)) / SR)
    for ot_ in events['ticks']:
        s = int(ot_ * SR)
        if s + len(click) < n:
            out[s:s + len(click)] += 0.34 * click
    if events['riser']:
        a, b = events['riser']
        s, e = int(a * SR), int(b * SR)
        m = np.arange(e - s) / max(1, e - s)
        sweep = np.sin(2 * np.pi * np.cumsum(110 + 900 * m ** 2) / SR)
        out[s:e] += (0.05 + 0.20 * m ** 2) * sweep + (0.02 + 0.12 * m ** 2) * band_noise(e - s, 600, 5000, rng)
    if events.get('hit'):
        s = int(events['hit'] * SR)
        L = int(2.2 * SR)
        tt = np.arange(L) / SR
        thump = np.sin(2 * np.pi * np.cumsum(95 * np.exp(-tt * 1.6) + 28) / SR) * np.exp(-tt * 1.4)
        burst = band_noise(L, 60, 3000, rng) * np.exp(-tt * 7)
        tail = np.convolve(burst, np.exp(-np.arange(int(0.9 * SR)) / (0.25 * SR)) * rng.standard_normal(int(0.9 * SR)), 'same')
        tail /= np.abs(tail).max() + 1e-9
        end = min(n, s + L)
        out[s:end] += (0.75 * thump + 0.25 * burst + 0.10 * tail * np.exp(-tt * 1.8))[:end - s]
    fade = np.minimum(1.0, np.minimum(t / 1.5, (total_s - t) / 2.5))
    out *= fade
    out = np.tanh(out * 1.4) * 0.75
    out /= max(1e-9, np.abs(out).max()) / 0.7
    right = np.concatenate([np.zeros(int(0.0006 * SR)), out[:-int(0.0006 * SR)]])
    pcm = (np.stack([out, right], axis=1) * 32767).astype(np.int16)
    with wave.open(path, 'wb') as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(SR)
        w.writeframes(pcm.tobytes())


# ------------------------------------------------------------------------------------------ main


def stills(out_dir):
    os.makedirs(out_dir, exist_ok=True)
    shots, have_c = build_shots()
    for n, s in enumerate(shots):
        times = shot_times(s)
        ev = dict(frames=0, ticks=[], air=[], hit=None, riser=None)
        keep = {}

        def sink(arr, keep=keep):
            keep['last'] = arr
            keep.setdefault('frames', []).append(arr)

        part = dict(s)
        part['b'] = s['a'] + min(len(times), 90) * s['speed'] / FPS
        render_shot(part, sink, ev, None)
        frames = keep['frames']
        for tag, f in (('early', frames[min(len(frames) - 1, 20)]), ('late', frames[-1])):
            Image.fromarray(f).save(os.path.join(out_dir, f'cine_shot{n}_{tag}.png'))
    print('stills done, take_C present:', have_c)


def full(out_path):
    shots, have_c = build_shots()
    tmp_v = out_path + '.video.mp4'
    enc = subprocess.Popen([FF, '-y', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{W}x{H}', '-r', str(FPS), '-i', '-', '-c:v', 'libx264',
                            '-preset', 'fast', '-threads', '6', '-crf', '17', '-pix_fmt', 'yuv420p', tmp_v], stdin=subprocess.PIPE)
    sink = lambda arr: enc.stdin.write(arr.tobytes())  # noqa: E731
    ev = dict(frames=0, ticks=[], air=[], hit=None, riser=None)

    def card(lines, secs, typed=False):
        n = int(secs * FPS)
        for k in range(n):
            sink(card_frame(lines, k, n, typed))
        ev['frames'] += n

    card([('PX4 offboard control', 58, FG, 470, 0.4), ('What happens when the command stream goes quiet?', 26, MUTED, 560, 2.4)], 5.0, typed=True)
    for n_, s in enumerate(shots):
        s['dip'] = n_ in (0, 3, 4, len(shots) - 1)
        render_shot(s, sink, ev, None)
        print('shot', n_, 'done', flush=True)
    card([('PX4 gives a companion computer 1.0 s of silence.', 40, FG, 330, 0.3),
          ('After that it takes the drone back.', 40, FG, 392, 1.4),
          ('With the timeout set to 0.5, 1.0 and 2.0 s, PX4 left offboard 0.57, 1.05 and 2.0 s after the stream stopped.', 22, MUTED, 560, 3.0),
          ('Simulation only: PX4 SITL with Gazebo, 2 to 3 runs per condition.', 22, MUTED, 604, 4.0)], 9.0)
    enc.stdin.close()
    enc.wait()
    total = ev['frames'] / FPS
    wav = out_path + '.wav'
    synth(ev, total, wav)
    tmp_out = out_path + '.part.mp4'
    subprocess.run([FF, '-y', '-i', tmp_v, '-i', wav, '-c:v', 'copy', '-c:a', 'aac', '-b:a', '192k', '-shortest', tmp_out], check=True,
                   capture_output=True)
    os.replace(tmp_out, out_path)
    for p in (tmp_v, wav):
        os.remove(p)
    print('done', out_path, f'{total:.1f} s', 'take_C used' if have_c else 'fallback C footage used')


if __name__ == '__main__':
    if sys.argv[1] == 'stills':
        stills(sys.argv[2])
    else:
        full(sys.argv[2])
