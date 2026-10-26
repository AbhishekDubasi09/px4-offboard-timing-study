"""Joystick or keyboard teleoperation: body-frame input -> NED velocity setpoints at 20 Hz.

source:=joy       reads sensor_msgs/Joy on /joy (ROS 2 joy package)
source:=keyboard  reads keys from the terminal: w/s forward/back, a/d left/right,
                  r/f up/down, q/e yaw left/right, space stop, x land
Deadzone and limits are applied to joystick axes; keys decay after `key_hold` seconds.
"""
import math
import select
import sys
import termios
import time
import tty

import rclpy
from sensor_msgs.msg import Joy

from px4_offboard_study.common import Px4Link, STEADY
from px4_offboard_study.mathutil import body_to_ned, shape

DT = 0.05


class Teleop(Px4Link):
    def __init__(self):
        super().__init__('teleop_node')
        p = self.declare_parameter
        self.source = p('source', 'joy').value
        self.max_h = p('max_horizontal', 2.0).value
        self.max_v = p('max_vertical', 1.0).value
        self.max_yaw = p('max_yaw_rate', 0.8).value
        self.deadzone = p('deadzone', 0.1).value
        self.key_hold = p('key_hold', 0.3).value
        self.alt = p('alt', 5.0).value
        self.axes = {'right': p('axis_right', 3).value, 'fwd': p('axis_fwd', 4).value,
                     'up': p('axis_up', 1).value, 'yaw': p('axis_yaw', 0).value}
        self.inp = [0.0, 0.0, 0.0, 0.0]  # forward, right, up, yaw rate (already scaled)
        self.last_in = 0.0
        self.yaw = math.nan
        self.warm = 0
        self.airborne = False
        self.is_landing = False
        if self.source == 'joy':
            self.create_subscription(Joy, '/joy', self.on_joy, 10)
        else:
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        self.create_timer(DT, self.tick, clock=STEADY)

    def on_joy(self, msg):
        def ax(i):
            return msg.axes[i] if i < len(msg.axes) else 0.0
        a = self.axes
        self.inp = [shape(ax(a['fwd']), self.deadzone, self.max_h), shape(ax(a['right']), self.deadzone, self.max_h),
                    shape(ax(a['up']), self.deadzone, self.max_v), shape(-ax(a['yaw']), self.deadzone, self.max_yaw)]
        self.last_in = time.monotonic()
        if msg.buttons and msg.buttons[0] and not self.is_landing:  # button 0 (A on most pads) lands
            self.is_landing = True
            self.land()

    def read_keys(self):
        while select.select([sys.stdin], [], [], 0)[0]:
            k = sys.stdin.read(1)
            h, v, y = self.max_h * 0.5, self.max_v, self.max_yaw
            table = {'w': (h, 0, 0, 0), 's': (-h, 0, 0, 0), 'd': (0, h, 0, 0), 'a': (0, -h, 0, 0),
                     'r': (0, 0, v, 0), 'f': (0, 0, -v, 0), 'e': (0, 0, 0, y), 'q': (0, 0, 0, -y),
                     ' ': (0, 0, 0, 0)}
            if k in table:
                self.inp = list(table[k])
                self.last_in = time.monotonic()
            elif k == 'x':
                self.is_landing = True
                self.land()

    def tick(self):
        if self.source == 'keyboard':
            self.read_keys()
        if self.pos is None or self.status is None:
            return
        if math.isnan(self.yaw):
            self.yaw = self.pos.heading
        timeout = 0.5 if self.source == 'joy' else self.key_hold
        if time.monotonic() - self.last_in > timeout:
            self.inp = [0.0, 0.0, 0.0, 0.0]
        if self.is_landing:
            return
        self.yaw += self.inp[3] * DT
        vn, ve, vd = body_to_ned(self.inp[0], self.inp[1], self.inp[2], self.yaw)
        if not self.airborne:
            # Position setpoint at the takeoff altitude: stream first, then switch and arm, then climb.
            self.send_heartbeat(False)
            self.send_setpoint(position=(0.0, 0.0, -self.alt), yaw=self.yaw)
            self.warm += 1
            if self.warm == 20:
                self.set_offboard()
                self.arm()
            if self.armed and self.pos.z < -self.alt + 0.3:
                self.airborne = True
            return
        self.send_heartbeat(True)
        self.send_setpoint(velocity=(vn, ve, vd), yaw=self.yaw)

    def destroy_node(self):
        if self.source == 'keyboard':
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        super().destroy_node()


def main():
    rclpy.init()
    node = Teleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
