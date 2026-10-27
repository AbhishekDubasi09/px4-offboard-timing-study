"""Quadcopter rotor sound synthesised from the flight log, so it follows the controls.

Model, kept deliberately physical:
- Thrust the vehicle needs is |g + a| from the logged acceleration (smoothed), so climbs, braking and turns change the sound.
- Each of the four motors has a blade-pass frequency 2 * rpm / 60 (two-blade props), with harmonics, a small
  shaft-rate roughness, a fixed detune between motors and a slow wobble that grows with control activity.
- Forward or sideways acceleration speeds up one pair of motors and slows the other (that is how a quad tilts), and yaw
  speeds one diagonal pair against the other.
- Broadband blade and air noise grows with thrust. The motors spool up at arming, and are silent when disarmed.
- Camera distance sets the level and, when far, rolls off the highs.
No ticks, risers or hits: nothing in the track is there that a real drone would not make.
"""
import wave

import numpy as np

SR = 48000
G = 9.81
RPM_HOVER = 5400.0
IDLE_C = 0.36
BLADES = 2


def smooth(x, n):
    n = max(1, int(n) | 1)
    return np.convolve(np.pad(x, (n // 2, n // 2), mode='edge'), np.ones(n) / n, 'valid')


def motor_signals(t, alt, vx, vy, vz, heading, armed):
    t = np.asarray(t, float).copy()
    if (np.diff(t) <= 0).any():
        t = t + np.arange(len(t)) * 1e-6
    vn, ve, vd = smooth(vx, 15), smooth(vy, 15), smooth(vz, 15)
    an, ae, ad = (smooth(np.gradient(v, t), 9) for v in (vn, ve, vd))
    c = np.clip(np.sqrt((1 - ad / G) ** 2 + (np.hypot(an, ae) / G) ** 2), 0.55, 1.9)
    on = np.asarray(armed) > 0.5
    ground = (np.asarray(alt) < 0.15) & (np.abs(vd) < 0.3)
    c = np.where(on & ground, IDLE_C, c)
    c = np.where(on, c, 0.0)
    h = np.asarray(heading, float)
    return dict(c=c, ax=an * np.cos(h) + ae * np.sin(h), ay=-an * np.sin(h) + ae * np.cos(h),
                yaw=smooth(np.gradient(np.unwrap(h), t), 9))


def band_noise(n, lo, hi, rng):
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / SR)
    spec *= ((f >= lo) & (f <= hi))
    x = np.fft.irfft(spec, n)
    return x / (np.abs(x).max() + 1e-9)


def lowpass(x, fc):
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / SR)
    spec *= 1.0 / (1.0 + (f / fc) ** 4)
    return np.fft.irfft(spec, len(x))


def render(rows, total_s, path, seed=5):
    """rows: array with columns output_time, thrust c, accel forward, accel right, yaw rate, armed, camera distance."""
    rng = np.random.default_rng(seed)
    n = int(total_s * SR)
    tt = np.arange(n) / SR
    ot = rows[:, 0]
    dt = 0.01
    grid = np.arange(0, total_s, dt)

    def on_grid(col, outside=0.0):
        return np.interp(grid, ot, rows[:, col], left=outside, right=outside)

    c, axb, ayb, yaw = (on_grid(i) for i in (1, 2, 3, 4))
    dist = on_grid(6, outside=5.0)
    lag, v = np.empty_like(c), 0.0     # motors take time to change speed
    for i, x in enumerate(c):
        v += (dt / (0.10 if x > v else 0.16)) * (x - v)
        lag[i] = v
    base = RPM_HOVER * np.sqrt(np.clip(lag, 0, None))
    k = 0.5 * 1.3 / G
    act = np.clip(np.hypot(axb, ayb) / 2.5 + np.abs(yaw) / 1.2, 0, 1)
    detune = [1.000, 1.013, 0.990, 1.007]
    s_ax, s_ay, s_yaw = [-1, -1, 1, 1], [1, -1, 1, -1], [1, -1, -1, 1]      # front-left, front-right, rear-left, rear-right
    pan = [(0.70, 0.30), (0.30, 0.70), (0.65, 0.35), (0.35, 0.65)]
    amps = np.array([1.0, 0.62, 0.40, 0.27, 0.18, 0.12, 0.08, 0.05])
    left, right = np.zeros(n), np.zeros(n)
    for m in range(4):
        w = smooth(rng.standard_normal(len(grid)), 21)
        w /= w.std() + 1e-9
        rpm = np.clip(base * detune[m] * (1 + k * (s_ax[m] * axb + s_ay[m] * ayb) + 0.04 * s_yaw[m] * yaw + (0.004 + 0.02 * act) * w), 0, 9000)
        freq = np.interp(tt, grid, rpm) * BLADES / 60.0
        ph = np.cumsum(2 * np.pi * freq / SR)
        sig = 0.35 * np.sin(0.5 * ph + rng.uniform(0, 6.28))
        for h, a in enumerate(amps, 1):
            sig += a * np.sin(h * ph + rng.uniform(0, 6.28))
        sig *= np.interp(tt, grid, (rpm / RPM_HOVER) ** 2.5)
        left += pan[m][0] * sig
        right += pan[m][1] * sig
    env = np.interp(tt, grid, (base / RPM_HOVER) ** 2.5)
    for ch in (left, right):
        ch += 1.1 * band_noise(n, 600, 6500, rng) * env + 0.6 * band_noise(n, 70, 300, rng) * env
    dist_s = np.interp(tt, grid, dist)
    gain = np.clip(2.8 / np.maximum(dist_s, 1.0), 0.2, 1.4)
    far = np.clip((dist_s - 3.5) / 8.0, 0, 1)
    out = []
    for ch in (left, right):
        out.append((ch * (1 - far) + lowpass(ch, 1400) * far) * gain)
    st = np.stack(out, axis=1)
    active = env > 0.15
    ref = np.percentile(np.abs(st[active][::5]), 80) if active.any() else 1.0
    st = np.tanh(st * (0.085 / max(ref, 1e-9)))   # about minus 22 dB RMS in flight, peaks well below full scale
    fade = np.minimum(1.0, np.minimum(tt / 0.25, (total_s - tt) / 0.5))
    pcm = (st * fade[:, None] * 32767).astype(np.int16)
    with wave.open(path, 'wb') as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(SR)
        wf.writeframes(pcm.tobytes())
