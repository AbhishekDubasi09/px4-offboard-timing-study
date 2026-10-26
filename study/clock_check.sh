#!/usr/bin/env bash
# Detect wall-clock steps relative to the monotonic clock over 60 s.
python3 - <<'PY'
import time
t0w, t0m = time.time(), time.monotonic()
last = 0.0
worst = 0.0
end = time.monotonic() + 60
while time.monotonic() < end:
    time.sleep(0.05)
    d = (time.time() - t0w) - (time.monotonic() - t0m)
    if abs(d - last) > 0.05:
        print(f'wall-clock step {d - last:+.3f} s at {time.monotonic() - t0m:.1f} s')
    last = d
    worst = max(worst, abs(d))
print('max drift vs monotonic: %.3f s' % worst)
PY
cat /sys/devices/system/clocksource/clocksource0/current_clocksource
