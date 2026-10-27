# results

Logs and figures behind FINDINGS.md. Every run has a CSV (100 Hz node log), a JSON (settings, measured real-time factor) and a log; each folder has `summary.csv` and `figures/`.

| Folder | What |
|---|---|
| `final/` | Clean demo flights (20 Hz stream, offboard never lost). Source of `fig0_demo_flight.png`. |
| `run2/` | Main sweeps: stream rate, single gap, jitter, CPU load. Parameter not pinned, real-time factor not measured (FINDINGS.md, section 3). |
| `run4/` | Simulation speed sweep (0.25x, 0.5x, 1.0x). |
| `run5/` | Timeout parameter test, `COM_OF_LOSS_T` 0.5 / 1.0 / 2.0 s, parameter pinned. |
| `showcase_raw/`, `cinematic_raw/` | Flight logs of the filmed takes. Raw screen recordings are not in git. |
| `videos/` | The finished videos (the only mp4 files tracked in git). |
| `cinema_work/` | Local scratch for the cinematic cut, ignored by git. |
