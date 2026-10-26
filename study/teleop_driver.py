"""Scripted joystick for the teleop check: publishes sensor_msgs/Joy and logs the vehicle state to CSV.

Usage (ROS env sourced): python3 teleop_driver.py <out.csv>
Timeline after start: 0-14 s the teleop node takes off and holds; then forward, right, up, yaw, down, land button.
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

from px4_offboard_study.common import PX4_QOS, TOPIC_LOCAL_POS, TOPIC_STATUS
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

# (start s, end s, label, axes dict). Axes follow the teleop defaults: yaw 0, up 1, right 3, forward 4.
PLAN = [(14, 18, 'forward', {4: 1.0}), (19, 23, 'right', {3: 1.0}), (24, 27, 'up', {1: 0.6}),
        (28, 32, 'yaw', {0: -1.0}), (33, 36, 'down', {1: -0.6})]
LAND_AT = 38.0


class Driver(Node):
    def __init__(self, path):
        super().__init__('teleop_driver')
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.create_subscription(VehicleLocalPosition, TOPIC_LOCAL_POS, lambda m: setattr(self, 'pos', m), PX4_QOS)
        self.create_subscription(VehicleStatus, TOPIC_STATUS, lambda m: setattr(self, 'st', m), PX4_QOS)
        self.pos = self.st = None
        self.f = open(path, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'stick', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'heading', 'nav_state', 'armed'])
        self.t0 = time.monotonic()
        self.create_timer(0.05, self.tick)

    def tick(self):
        t = time.monotonic() - self.t0
        axes, label, buttons = [0.0] * 6, 'none', [0]
        for a, b, name, d in PLAN:
            if a <= t < b:
                label = name
                for i, v in d.items():
                    axes[i] = v
        if t >= LAND_AT:
            buttons, label = [1], 'land'
        m = Joy()
        m.axes, m.buttons = axes, buttons
        self.pub.publish(m)
        p, s = self.pos, self.st
        self.w.writerow([f'{t:.3f}', label] + ([p.x, p.y, p.z, p.vx, p.vy, p.vz, p.heading] if p else [''] * 7)
                        + [s.nav_state if s else '', int(bool(s and s.arming_state == 2))])
        if t > LAND_AT + 14:
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
