"""Scripted joystick for the teleop acts: publishes sensor_msgs/Joy and logs sticks and vehicle state to CSV.

Usage (ROS env sourced): python3 teleop_driver.py <out.csv>
The teleop node takes off by itself (about 10 s). Then smooth, human-like stick motions follow, then the land button.
Axes follow the teleop defaults: yaw 0, up 1, right 3, forward 4 (button 0 = land). Values are what a real pad sends:
stick pushed left or up = positive, right = negative for the horizontal axes.
"""
import csv
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from px4_offboard_study.common import PX4_QOS, TOPIC_LOCAL_POS, TOPIC_STATUS
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

LAND_AT = 52.0
END_AT = LAND_AT + 14.0


def pulse(t, t0, dur, amp):
    """One smooth half-sine push of the stick: 0 -> amp -> 0."""
    if t0 <= t < t0 + dur:
        return amp * math.sin(math.pi * (t - t0) / dur)
    return 0.0


def sticks(t):
    """Returns (label, yaw, up, right, forward) for time t after the node started."""
    fwd = pulse(t, 14.0, 4.5, 0.95)
    right = pulse(t, 19.5, 4.5, 0.95)
    up = pulse(t, 25.0, 3.5, 0.65) - pulse(t, 29.0, 3.5, 0.65)
    yaw, label = 0.0, 'idle'
    if 33.5 <= t < 45.0:  # carving turn: forward stick held, yaw stick pushed
        fwd = 0.85 * min(1.0, (t - 33.5) / 1.0) * min(1.0, (45.0 - t) / 1.0)
        yaw = -0.55 * min(1.0, (t - 33.5) / 1.0) * min(1.0, (45.0 - t) / 1.0)
        label = 'carve'
    elif fwd > 0.02:
        label = 'forward'
    elif right > 0.02:
        label = 'right'
    elif up > 0.02:
        label = 'climb'
    elif up < -0.02:
        label = 'descend'
    return label, yaw, up, right, fwd


class Driver(Node):
    def __init__(self, path):
        super().__init__('teleop_driver')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(VehicleLocalPosition, TOPIC_LOCAL_POS, lambda m: setattr(self, 'pos', m), PX4_QOS)
        self.create_subscription(VehicleStatus, TOPIC_STATUS, lambda m: setattr(self, 'st', m), PX4_QOS)
        self.pos = self.st = None
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'label', 'ax_yaw', 'ax_up', 'ax_right', 'ax_fwd', 'btn_land',
                         'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'nav_state', 'armed'])
        self.t0 = time.monotonic()
        self.create_timer(0.02, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        label, yaw, up, right, fwd = sticks(t)
        btn = 1 if t >= LAND_AT else 0
        if btn:
            label = 'land'
        axes = [0.0] * 6
        axes[0], axes[1], axes[3], axes[4] = yaw, up, -right, fwd  # ROS joy: stick pushed right/left = negative/positive
        m = Joy()
        m.axes, m.buttons = axes, [btn]
        self.pub.publish(m)
        p, s = self.pos, self.st
        self.w.writerow([f'{t:.3f}', label, f'{yaw:.3f}', f'{up:.3f}', f'{-right:.3f}', f'{fwd:.3f}', btn]
                        + ([p.x, p.y, p.z, p.vx, p.vy, p.vz, p.heading] if p else [''] * 7)
                        + [s.nav_state if s else '', int(bool(s and s.arming_state == 2))])
        if t > END_AT:
            self.f.close()
            raise SystemExit


def main():
    rclpy.init()
    n = Driver(sys.argv[1])
    try:
        rclpy.spin(n)
    except SystemExit:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
