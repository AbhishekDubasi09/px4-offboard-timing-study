"""Offboard flight (arm, takeoff, hold, turn, velocity square, land) with a degradable setpoint stream.

The control loop ticks at 100 Hz and logs every tick. The setpoint stream (heartbeat + setpoint)
is sent at `rate_hz` with optional Gaussian `jitter_std` and one dropped window
(`gap_start` s after the track phase begins, `gap_len` s long). With defaults it is the plain demo.
"""
import csv
import faulthandler
import json
import math
import random
import time

import rclpy

from px4_offboard_study.common import Px4Link, NAV_OFFBOARD, STEADY

TICK = 0.01
SQUARE = [(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)]
SEG_S = 4.0


class StudyNode(Px4Link):
    def __init__(self):
        super().__init__('study_node')
        p = self.declare_parameter
        self.alt = p('alt', 5.0).value
        self.rate_hz = p('rate_hz', 20.0).value
        self.jitter_std = p('jitter_std', 0.0).value
        self.gap_start = p('gap_start', -1.0).value
        self.gap_len = p('gap_len', 0.0).value
        self.log_path = p('log_path', 'run.csv').value
        self.seed = p('seed', 0).value
        random.seed(self.seed)

        self.phase = 'warmup'
        self.phase_t0 = time.monotonic()
        self.track_t0 = None
        self.next_send = 0.0
        self.n_sent = 0
        self.n_warm = 0
        self.yaw_target = 0.0
        self.was_offboard = False
        self.exit_time = None
        self.exit_nav = None
        self.t_start = time.monotonic()
        self.cmd = (0.0, 0.0)
        self.sent_tick = 0

        self.f = open(self.log_path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'phase', 'sent', 'cmd_vn', 'cmd_ve', 'x', 'y', 'z', 'vx', 'vy', 'vz',
                         'heading', 'nav_state', 'armed', 'px4_ts_us', 'wall_us'])
        self.stall_f = open(self.log_path.replace('.csv', '.stall'), 'w')  # stack dump if a tick takes > 1 s
        self.create_timer(TICK, self.tick, clock=STEADY)

    def set_phase(self, name):
        self.phase = name
        self.phase_t0 = time.monotonic()
        self.get_logger().info(f'phase -> {name}')

    def in_gap(self, now):
        if self.track_t0 is None or self.gap_start < 0:
            return False
        t = now - self.track_t0
        return self.gap_start <= t < self.gap_start + self.gap_len

    def stream(self, now):
        """Send heartbeat + setpoint when due. Returns 1 if a message pair was sent."""
        if now < self.next_send or self.in_gap(now):
            return 0
        z = -self.alt
        velocity = self.phase == 'track'
        self.send_heartbeat(velocity)
        if velocity:
            self.send_setpoint(velocity=(self.cmd[0], self.cmd[1], 0.0), yaw=self.yaw_target)
        else:
            self.send_setpoint(position=(0.0, 0.0, z), yaw=self.yaw_target)
        # Anchor the schedule on the previous due time, not on `now`, so the achieved rate equals
        # rate_hz instead of being quantised by the 10 ms tick. After a stall or gap, resume at once.
        period = 1.0 / self.rate_hz
        step = max(0.001, random.gauss(period, self.jitter_std)) if self.jitter_std else period
        self.next_send = max(self.next_send + step, now)
        self.n_sent += 1
        self.sent_tick = 1
        return 1

    def tick(self):
        now = time.monotonic()
        el = now - self.phase_t0
        pos = self.pos
        self.sent_tick = 0
        faulthandler.dump_traceback_later(1.0, file=self.stall_f)  # re-armed every tick

        # Warm-up: stream at full rate before the mode switch, then switch and arm.
        if self.phase == 'warmup':
            saved = self.rate_hz
            self.rate_hz = max(saved, 20.0)
            self.n_warm += self.stream(now)
            self.rate_hz = saved
            if self.n_warm >= 20 and pos is not None and self.status is not None:
                self.set_offboard()
                self.arm()
                self.set_phase('takeoff')
        elif self.phase == 'takeoff':
            self.stream(now)
            if self.nav_state != NAV_OFFBOARD and el > 2.0:
                self.set_offboard()
                self.arm()
            if pos is not None and pos.z < -self.alt + 0.3:
                self.set_phase('hold')
        elif self.phase == 'hold':
            self.stream(now)
            if el > 3.0:
                self.yaw_target = math.pi / 2
                self.set_phase('turn')
        elif self.phase == 'turn':
            self.stream(now)
            if el > 4.0:
                self.track_t0 = now
                self.set_phase('track')
        elif self.phase == 'track':
            seg = int(el // SEG_S)
            if seg >= len(SQUARE):
                self.cmd = (0.0, 0.0)
                self.set_phase('settle')
            else:
                self.cmd = SQUARE[seg]
                self.stream(now)
        elif self.phase == 'settle':
            self.stream(now)
            if el > 2.0:
                self.land()
                self.set_phase('land')
        elif self.phase == 'land':
            if (not self.armed and el > 3.0) or el > 40.0:
                self.finish()
                return

        # Offboard exit detection (failsafe or otherwise) after it was entered.
        if self.nav_state == NAV_OFFBOARD:
            self.was_offboard = True
        elif self.was_offboard and self.exit_time is None and self.phase in ('hold', 'turn', 'track', 'settle'):
            self.exit_time = now - self.t_start
            self.exit_nav = self.nav_state
            self.get_logger().warning(f'left offboard (nav_state {self.nav_state}) during {self.phase}')
            self.land()
            self.set_phase('land')

        if self.phase == 'warmup' and el > 60.0:
            self.get_logger().error('timeout: no PX4 data')
            self.finish()
            return

        p = pos
        self.w.writerow([f'{now - self.t_start:.4f}', self.phase, self.sent_tick, self.cmd[0], self.cmd[1],
                         p.x if p else '', p.y if p else '', p.z if p else '',
                         p.vx if p else '', p.vy if p else '', p.vz if p else '',
                         p.heading if p else '', self.nav_state, int(self.armed),
                         p.timestamp if p else '', self.now_us()])

    def finish(self):
        self.f.close()
        result = {
            'rate_hz': self.rate_hz, 'jitter_std': self.jitter_std, 'gap_start': self.gap_start,
            'gap_len': self.gap_len, 'seed': self.seed, 'messages_sent': self.n_sent,
            'gap_onset_s': (self.track_t0 - self.t_start + self.gap_start)
            if self.track_t0 is not None and self.gap_start >= 0 else None,
            'offboard_lost': self.exit_time is not None,
            'exit_time_s': self.exit_time, 'exit_nav_state': self.exit_nav,
            'completed_track': self.track_t0 is not None,
        }
        with open(self.log_path.replace('.csv', '.json'), 'w') as fh:
            json.dump(result, fh, indent=2)
        self.get_logger().info(f'result: {result}')
        raise SystemExit


def main():
    rclpy.init()
    node = StudyNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
