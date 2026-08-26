#!/usr/bin/env python3
"""Command arbiter: the ONLY node in this project that publishes /cmd_vel.

Why this node exists
--------------------
Several things want to drive the car: a joystick on a phone, an autonomous
behaviour, a safety watchdog, a person hitting the kill switch. If each of them
published /cmd_vel directly, the VESC node would receive their messages
interleaved at whatever rate each happened to run, and "stop" and "follow" would
alternate several times a second. That is not a bug you can debug from the
outside; it looks like the car twitching.

So every source publishes to its own topic and this node decides which one wins,
then speaks to the actuator with one voice.

Priority, highest first:

    1. /estop        (std_msgs/Bool)   latched kill switch
    2. /safety_cmd   (Twist)           watchdogs, e.g. LiDAR proximity
    3. /teleop_cmd   (Twist)           manual joystick stream
    4. /behavior_cmd (Twist)           autonomous primitives

Two safety properties are built in rather than bolted on:

**Staleness.** Every source has a timeout. A source that stops publishing stops
winning, so a crashed behaviour node or a phone that walked out of wifi range
releases control instead of latching its last command forever. If no source is
fresh, the output is zero.

**Streaming output.** /cmd_vel is published on every tick, not only when the
command changes. The downstream base controller times out on missing messages
and zeroes the motors, and that timeout is the point: it is what stops the car if
*this* node dies. Publishing continuously is what keeps that safety net armed
during normal operation instead of tripping constantly.

Output is also clamped to the car's calibrated limits, so a typo in a manual
`ros2 topic pub` cannot command full throttle.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String


class Source:
    """One prioritised input: its latest Twist and when it arrived."""

    def __init__(self, name, timeout_s):
        self.name = name
        self.timeout_s = timeout_s
        self.twist = Twist()
        self.stamp = None          # None means "never heard from"

    def update(self, twist, now_s):
        self.twist = twist
        self.stamp = now_s

    def is_fresh(self, now_s):
        if self.stamp is None:
            return False
        return (now_s - self.stamp) <= self.timeout_s


class ArbiterNode(Node):

    def __init__(self):
        super().__init__('arbiter_node')

        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('safety_timeout_s', 0.5)
        self.declare_parameter('teleop_timeout_s', 0.5)
        self.declare_parameter('behavior_timeout_s', 1.0)
        # Ceilings on what any source is allowed to ask for.
        #
        # Units, because they are NOT the same as the calibration file's despite
        # sharing a name. vesc_twist_node reads ros_racer_calibration.yaml and
        # computes  max_rpm = max_throttle * max_rpm = 0.382 * 20000 = 7640 ERPM,
        # then commands  rpm = 7640 * msg.linear.x.  So Twist linear.x is a
        # FRACTION of that already-capped range, in [-1, 1], and our max_throttle
        # is a second, project-level speed limit on top of the car's.
        #
        # MEASURED on the bench 2026-08-25 with tools/vesc_probe.py: the motor
        # does not start at 500 ERPM and does start at 1000, so the practical
        # floor is linear.x ~= 0.13. The class's own lane_guidance_node drives
        # between 0.363 and 0.382 (2773 to 2918 ERPM), which is why 0.382 is the
        # right ceiling: it matches what the car is calibrated to do.
        #
        # A behaviour that wants to go slowly cannot simply scale throttle toward
        # zero. Below the floor the car does not creep, it stops.
        self.declare_parameter('max_steering', 0.8)
        self.declare_parameter('max_throttle', 0.382)
        # Informational, published in status so behaviours can respect it.
        self.declare_parameter('throttle_floor', 0.25)

        self.publish_hz = self.get_parameter('publish_hz').value
        self.max_steering = self.get_parameter('max_steering').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.throttle_floor = self.get_parameter('throttle_floor').value

        # Highest priority first. Order in this list IS the priority order.
        self.sources = [
            Source('safety', self.get_parameter('safety_timeout_s').value),
            Source('teleop', self.get_parameter('teleop_timeout_s').value),
            Source('behavior', self.get_parameter('behavior_timeout_s').value),
        ]
        self.by_name = {s.name: s for s in self.sources}

        self.estop = False
        self.active = None          # name of the winning source, for logging

        self.create_subscription(Bool, 'estop', self._on_estop, 10)
        self.create_subscription(
            Twist, 'safety_cmd', lambda m: self._on_cmd('safety', m), 10)
        self.create_subscription(
            Twist, 'teleop_cmd', lambda m: self._on_cmd('teleop', m), 10)
        self.create_subscription(
            Twist, 'behavior_cmd', lambda m: self._on_cmd('behavior', m), 10)

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'arbiter/status', 10)

        self.create_timer(1.0 / self.publish_hz, self._tick)

        self.get_logger().info(
            f'arbiter up: {self.publish_hz:.0f} Hz, '
            f'steering +/-{self.max_steering}, throttle +/-{self.max_throttle}, '
            f'measured floor {self.throttle_floor}')

    # -- inputs ---------------------------------------------------------------

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _on_estop(self, msg):
        if msg.data != self.estop:
            self.get_logger().warn(
                'E-STOP ENGAGED' if msg.data else 'e-stop cleared')
        self.estop = msg.data

    def _on_cmd(self, name, msg):
        self.by_name[name].update(msg, self._now())

    # -- output ---------------------------------------------------------------

    def _clamp(self, twist):
        out = Twist()
        out.linear.x = max(-self.max_throttle,
                           min(self.max_throttle, twist.linear.x))
        out.angular.z = max(-self.max_steering,
                            min(self.max_steering, twist.angular.z))
        return out

    def _select(self):
        """Return (name, twist) for whoever currently owns the actuators."""
        if self.estop:
            return 'estop', Twist()
        now = self._now()
        for src in self.sources:
            if src.is_fresh(now):
                return src.name, self._clamp(src.twist)
        return 'idle', Twist()

    def _tick(self):
        name, twist = self._select()

        if name != self.active:
            self.get_logger().info(f'control -> {name}')
            self.active = name

        self.cmd_pub.publish(twist)
        self.status_pub.publish(String(data=json.dumps({
            'active': name,
            'estop': self.estop,
            'throttle': round(twist.linear.x, 4),
            'steering': round(twist.angular.z, 4),
        })))

    def stop(self):
        """Best-effort zero on the way out."""
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ArbiterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Publishing a stop and immediately destroying the node is a race:
        # publish() hands off to DDS and returns before delivery. Stopping here,
        # outside the signal handler, and destroying in `finally` gives the
        # message a chance to actually leave.
        node.stop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
