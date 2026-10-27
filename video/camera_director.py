"""Scripted camera operator for the Gazebo GUI. Run inside Ubuntu WSL next to a flight.

Usage: python3 camera_director.py <A|C> <csv path>
It waits for the flight log to appear (the node creates it at its t=0), then moves the GUI camera on a timed plan,
updating 30 times per second so the motion is smooth. A = joystick flight, C = stream paused for 1.5 s.
Gazebo axes: x east, y north, z up. The drone spawns on the pad at the origin facing east.
"""
import math
import os
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.gui_camera_pb2 import GUICamera
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.vector3d_pb2 import Vector3d
from gz.transport13 import Node

node = Node()
state = {'mode': None}


def smooth(a, b, t):
    """Smoothstep from a to b as t goes 0 -> 1."""
    t = min(max(t, 0.0), 1.0)
    return a + (b - a) * (t * t * (3 - 2 * t))


def lerp3(p, q, t):
    return tuple(smooth(a, b, t) for a, b in zip(p, q))


def follow(name):
    req = StringMsg()
    req.data = name
    node.request('/gui/follow', req, StringMsg, Boolean, 500)


def set_offset(v):
    req = Vector3d()
    req.x, req.y, req.z = v
    node.request('/gui/follow/offset', req, Vector3d, Boolean, 500)


def set_pose(pos, target):
    """Static camera at pos looking at target. Camera looks along +x; positive pitch looks down."""
    dx, dy, dz = target[0] - pos[0], target[1] - pos[1], target[2] - pos[2]
    yaw = math.atan2(dy, dx)
    pitch = -math.atan2(dz, math.hypot(dx, dy))
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    cam = GUICamera()
    cam.pose.position.x, cam.pose.position.y, cam.pose.position.z = pos
    cam.pose.orientation.w = cy * cp
    cam.pose.orientation.x = -sy * sp
    cam.pose.orientation.y = cy * sp
    cam.pose.orientation.z = sy * cp
    node.request('/gui/move_to/pose', cam, GUICamera, Boolean, 500)


def mode(m):
    if state['mode'] != m:
        follow('x500_0' if m == 'follow' else '')
        state['mode'] = m


def climb_z(t):
    """Expected drone altitude during takeoff (from the logged flights)."""
    return smooth(0.45, 4.8, (t - 4.2) / 5.3)


def plan_A(t):
    if t < 11.0:   # low static camera, tilting up as the drone climbs out of the pad
        mode('static')
        set_pose((-2.7, -0.9, 0.22), (0.0, 0.0, max(0.5, climb_z(t))))
    elif t < 24.0:  # chase from behind (the drone faces east)
        mode('follow')
        set_offset((-2.6, 0.15, 0.75))
    elif t < 40.0:  # orbit during the carving turn
        mode('follow')
        th = math.radians(200 + 22 * (t - 24.0))
        set_offset((3.2 * math.cos(th), 3.2 * math.sin(th), 0.9))
    elif t < 52.0:  # almost top-down
        mode('follow')
        set_offset((0.8, 0.0, 8.0))
    else:           # low from the side for the landing
        mode('follow')
        set_offset((0.0, -4.5, 0.55))


def plan_C(t):
    mode('follow')
    if t < 0.35:   # marker: a brief jump the edit uses to find the sync point
        set_offset((0.0, 0.0, 45.0))
        return
    if t < 8.0:
        set_offset((-9.0, -9.0, 5.5))
    elif t < 13.0:
        set_offset(lerp3((-9.0, -9.0, 5.5), (-4.5, -4.5, 1.8), (t - 8.0) / 5.0))
    elif t < 16.5:
        set_offset(lerp3((-4.5, -4.5, 1.8), (0.0, -4.5, 0.9), (t - 13.0) / 3.5))
    elif t < 20.6:  # slow push-in toward the pause and the timeout
        set_offset(lerp3((0.0, -4.5, 0.9), (-1.3, -2.4, 0.7), (t - 16.5) / 4.1))
    else:           # pull back and up after the failsafe
        set_offset(lerp3((-1.3, -2.4, 0.7), (-6.0, -6.0, 2.8), (t - 20.6) / 6.4))


def main():
    plan = {'A': plan_A, 'C': plan_C}[sys.argv[1]]
    path = sys.argv[2]
    while not os.path.exists(path):   # the node creates its log at t = 0
        time.sleep(0.02)
    t0 = time.monotonic()
    end = 72.0 if sys.argv[1] == 'A' else 40.0
    while True:
        t = time.monotonic() - t0
        if t > end:
            break
        plan(t)
        time.sleep(1 / 30)
    print('director done', sys.argv[1], flush=True)


if __name__ == '__main__':
    main()
