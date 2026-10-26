"""Shared PX4 offboard plumbing: QoS, publishers, vehicle state, commands."""
import math
import time

from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

# PX4 uXRCE-DDS topics are best effort / volatile. A default (reliable) profile on the
# ROS 2 side leaves the endpoints unmatched and no messages arrive.
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Topic names carry the message version of the PX4 build (check with: ros2 topic list -t).
TOPIC_COMMAND = '/fmu/in/vehicle_command_v1'
TOPIC_LOCAL_POS = '/fmu/out/vehicle_local_position_v1'
TOPIC_STATUS = '/fmu/out/vehicle_status_v5'

NAV_OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMED = VehicleStatus.ARMING_STATE_ARMED
# rclpy timers follow the wall clock by default. In WSL2 the wall clock is stepped back by Hyper-V time sync
# (about 2.5 s every 32 s here), which freezes those timers; the steady clock is immune.
STEADY = Clock(clock_type=ClockType.STEADY_TIME)
NAN3 = [math.nan] * 3


class Px4Link(Node):
    """Node with the PX4 publishers, subscribers and command helpers."""

    def __init__(self, name):
        super().__init__(name)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.cmd_pub = self.create_publisher(VehicleCommand, TOPIC_COMMAND, PX4_QOS)
        self.create_subscription(VehicleLocalPosition, TOPIC_LOCAL_POS, self._on_pos, PX4_QOS)
        self.create_subscription(VehicleStatus, TOPIC_STATUS, self._on_status, PX4_QOS)
        self.pos = None
        self.status = None
        self._anchor = None

    def _on_pos(self, msg):
        self.pos = msg
        self._anchor = (msg.timestamp, time.monotonic())

    def _on_status(self, msg):
        self.status = msg

    def now_us(self):
        """Timestamp for outgoing messages. Anchored on the newest PX4 timestamp and advanced with the
        steady clock, so a wall-clock step cannot make the stream look stale to PX4. Before the first
        PX4 message there is no anchor and the wall clock is used."""
        if self._anchor is None:
            return int(self.get_clock().now().nanoseconds / 1000)
        ts, mono = self._anchor
        return int(ts + (time.monotonic() - mono) * 1e6)

    @property
    def nav_state(self):
        return self.status.nav_state if self.status else -1

    @property
    def armed(self):
        return bool(self.status and self.status.arming_state == ARMED)

    def send_heartbeat(self, velocity):
        """OffboardControlMode: the proof-of-life PX4 expects at more than 2 Hz."""
        m = OffboardControlMode()
        m.timestamp = self.now_us()
        m.position = not velocity
        m.velocity = velocity
        self.mode_pub.publish(m)

    def send_setpoint(self, position=None, velocity=None, yaw=math.nan):
        m = TrajectorySetpoint()
        m.timestamp = self.now_us()
        m.position = [float(v) for v in position] if position is not None else NAN3
        m.velocity = [float(v) for v in velocity] if velocity is not None else NAN3
        m.yaw = float(yaw)
        self.sp_pub.publish(m)

    def command(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = cmd
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def arm(self):
        self.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard(self):
        self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land(self):
        self.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
