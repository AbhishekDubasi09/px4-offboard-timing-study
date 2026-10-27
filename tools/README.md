# Debug scripts

One-off scripts used while tracking down problems (clock steps in WSL, PX4 timeouts and real-time factor, topic names,
GUI rendering). They are kept because FINDINGS.md refers to what they showed, but they are not part of the normal workflow.
Run them inside Ubuntu WSL. `px4_clock_test.sh` (PX4 clock against the monotonic clock) and `clock_check.sh` (wall-clock
steps) are the most useful ones.
