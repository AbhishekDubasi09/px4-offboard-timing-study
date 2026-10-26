"""Pure maths for the teleop node (no ROS imports, so it can be unit-tested anywhere)."""
import math


def shape(v, deadzone, limit):
    """Deadzone with rescale, then scale to the speed limit. Input and output are symmetric."""
    if abs(v) < deadzone:
        return 0.0
    v = math.copysign((abs(v) - deadzone) / (1.0 - deadzone), v)
    return max(-1.0, min(1.0, v)) * limit


def body_to_ned(fwd, right, up, yaw):
    """Body frame (forward, right, up) to NED (north, east, down) for heading `yaw` (rad, 0 = north, clockwise)."""
    vn = fwd * math.cos(yaw) - right * math.sin(yaw)
    ve = fwd * math.sin(yaw) + right * math.cos(yaw)
    return vn, ve, -up
