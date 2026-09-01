#!/usr/bin/env python3
"""LiDAR proximity guard: /scan -> /safety_cmd.

The camera sees roughly 70 degrees. The LD06, mounted as the highest item on the
car with an unobstructed view, sees 360. This node exists for the other 290: the
world the camera cannot look at, which is most of it.

Three cases in this project specifically:

* **Reversing.** The camera faces forward. When the car backs up, nothing else
  is watching where it is going.
* **Obstacles while following.** The car is steering to keep a person centred and
  is therefore, by construction, looking at the person rather than at the chair
  leg it is about to clip.
* **During a pan-servo search sweep**, when the camera is deliberately aimed away.

Publishes a zero Twist on /safety_cmd while a hazard is present, and **nothing at
all** when clear. That matters: the arbiter ranks safety above teleop and
behaviour, so a node that published continuously would veto everything forever.
Silence plus the arbiter's staleness timeout is what makes the veto momentary.

**The forward stop distance must sit inside the follow standoff.** Follow holds
1 m; if this node stopped at anything within 1 m it would fight the follow
controller permanently, since the person being followed is an obstacle from the
LiDAR's point of view. Sides and rear can be much tighter because nothing else
covers them.

**Sector windows depend on what the car is doing.** Reversing cares about behind,
driving cares about ahead, and applying a rear threshold while driving forward
would stop the car for a wall it is leaving.

**Direction of travel is read from the arbiter's INPUTS, not from /cmd_vel.**
Using the output would be circular: the guard vetoes /cmd_vel, so /cmd_vel goes
to zero, so the guard concludes the car is stationary, releases the veto, and the
car lurches forward again. Watching /behavior_cmd and /teleop_cmd sees what the
car is *trying* to do, which is unaffected by the veto and is the question that
actually matters.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        # Which way the LiDAR thinks "forward" is. Mounting rotation is a
        # physical fact nobody writes down, so it is a parameter and
        # tools/lidar_sectors.py finds it empirically.
        self.declare_parameter('forward_offset_deg', 0.0)

        self.declare_parameter('front_halfwidth_deg', 40.0)
        self.declare_parameter('rear_halfwidth_deg', 40.0)

        # Inside the follow standoff (1.0 m) on purpose, or the guard and the
        # follow controller deadlock over the person being followed.
        self.declare_parameter('front_stop_m', 0.45)
        self.declare_parameter('rear_stop_m', 0.35)
        self.declare_parameter('side_stop_m', 0.25)

        # A single stray return should not stop the car. Spring-2023 Team 10
        # documented open areas producing phantom close points on this class of
        # sensor, which made their robot freeze.
        self.declare_parameter('min_points', 3)
        self.declare_parameter('publish_hz', 20.0)
        # How long an intent stays live after the last command. Slightly longer
        # than the arbiter's own staleness window so the guard does not release
        # a moment before the car actually stops.
        self.declare_parameter('intent_timeout_s', 0.7)

        # HYSTERESIS. A single threshold on a noisy range estimate chatters:
        # an object sitting near the stop distance dithers across it and the
        # guard toggles at the tick rate, chopping a 3 second command into
        # fragments. Measured on the car 2026-09-01: 16 stop/clear cycles in one
        # minute. So a sector that is ALREADY blocking must clear by an extra
        # margin before it is believed, and a block is held for a minimum time.
        #
        # Both of these only ever DELAY a release. The distance at which the
        # guard stops the car is unchanged, so this cannot make it intervene
        # later than before, only let go later. That is the safe direction.
        self.declare_parameter('release_margin_m', 0.07)
        self.declare_parameter('min_block_s', 0.4)

        # Returns closer than this are discarded as the vehicle seeing itself.
        # Default 0.02 keeps the previous behaviour. Raise it to about 0.10 once
        # you know what is inside the car's own envelope: on this chassis the
        # LiDAR sits roughly 0.1 m from the bodywork, so anything nearer than
        # that is a cable or a bracket, not an obstacle. Do NOT raise it blind.
        self.declare_parameter('min_range_m', 0.02)

        # OVERRIDE. On a stand the wheels are off the ground and the guard is
        # measuring the bench, a chair leg, someone's foot: obstacles that are
        # real but irrelevant, and it vetoes every command because of them.
        #
        # Two rules make this an override rather than an off switch:
        #   * it defaults to ON, so a restart, a crash or a fresh page load can
        #     never silently leave the car unguarded;
        #   * the guard keeps RUNNING while overridden. It still computes and
        #     reports what it would have done, so the phone can show "would be
        #     stopping" and the operator knows what they are overriding. Only
        #     the veto is withheld.
        self.declare_parameter('enabled_on_start', True)
        self.declare_parameter('disabled_warn_s', 10.0)

        g = self.get_parameter
        self.offset = math.radians(float(g('forward_offset_deg').value))
        self.front_hw = math.radians(float(g('front_halfwidth_deg').value))
        self.rear_hw = math.radians(float(g('rear_halfwidth_deg').value))
        self.front_stop = float(g('front_stop_m').value)
        self.rear_stop = float(g('rear_stop_m').value)
        self.side_stop = float(g('side_stop_m').value)
        self.min_points = int(g('min_points').value)
        self.hz = float(g('publish_hz').value)
        self.intent_timeout = float(g('intent_timeout_s').value)
        self.release_margin = float(g('release_margin_m').value)
        self.min_block_s = float(g('min_block_s').value)
        self.min_range = float(g('min_range_m').value)

        self.sectors = {'front': 99.0, 'rear': 99.0, 'left': 99.0, 'right': 99.0}
        self.intent = 0.0            # throttle the car is TRYING to apply
        self.intent_at = 0.0
        self.blocked = None
        self.blocked_since = 0.0
        self.enabled = bool(g('enabled_on_start').value)
        self.warn_every = float(g('disabled_warn_s').value)
        self._last_warn = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'safety_cmd', 10)
        self.state_pub = self.create_publisher(String, 'obstacle_state', 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        # The arbiter's inputs, not its output. See the circularity note above.
        self.create_subscription(Twist, 'behavior_cmd', self._on_intent, 10)
        self.create_subscription(Twist, 'teleop_cmd', self._on_intent, 10)
        self.create_subscription(Bool, 'safety_enabled', self._on_enable, 10)
        self.create_timer(1.0 / self.hz, self._tick)

        self.get_logger().info(
            f'safety: front {self.front_stop:.2f} m, rear {self.rear_stop:.2f} m, '
            f'side {self.side_stop:.2f} m, forward offset '
            f'{math.degrees(self.offset):.0f} deg')

    # -- inputs ---------------------------------------------------------------

    def _on_intent(self, msg):
        """What the car is TRYING to do, so the right sector is watched."""
        if abs(msg.linear.x) > 0.01:
            self.intent = msg.linear.x
            self.intent_at = time.time()

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _on_scan(self, scan):
        near = {'front': [], 'rear': [], 'left': [], 'right': []}
        rmin = max(scan.range_min, self.min_range)

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < rmin or r > scan.range_max:
                continue
            # Angle relative to the car's forward direction.
            a = self._wrap(scan.angle_min + i * scan.angle_increment - self.offset)
            if abs(a) <= self.front_hw:
                near['front'].append(r)
            elif abs(abs(a) - math.pi) <= self.rear_hw:
                near['rear'].append(r)
            elif a > 0:
                near['left'].append(r)
            else:
                near['right'].append(r)

        for name, vals in near.items():
            if len(vals) < self.min_points:
                self.sectors[name] = 99.0
                continue
            # Nth smallest rather than the minimum: a lone spurious return
            # cannot trip the guard, but a real object spanning several beams
            # will.
            vals.sort()
            self.sectors[name] = vals[self.min_points - 1]

    # -- output ---------------------------------------------------------------

    def _hazard(self):
        """Which sector, if any, currently justifies stopping the car."""
        if time.time() - self.intent_at > self.intent_timeout:
            return None                      # nothing is trying to move

        if self.intent > 0 and self.sectors['front'] < self._limit('front'):
            return 'front'
        if self.intent < 0 and self.sectors['rear'] < self._limit('rear'):
            return 'rear'
        # Flanks matter whenever moving at all: the car steers as it drives, so a
        # wall alongside becomes a wall ahead a moment later.
        for side in ('left', 'right'):
            if self.sectors[side] < self._limit(side):
                return side
        return None

    def _limit(self, name):
        """Stop distance for a sector, widened while that sector is blocking."""
        base = {'front': self.front_stop,
                'rear': self.rear_stop}.get(name, self.side_stop)
        return base + self.release_margin if self.blocked == name else base

    def _on_enable(self, msg):
        want = bool(msg.data)
        if want == self.enabled:
            return
        self.enabled = want
        if want:
            self.get_logger().warn('LiDAR guard RE-ENABLED')
        else:
            self.get_logger().warn(
                'LiDAR guard OVERRIDDEN: obstacles will no longer stop the car')
        self._last_warn = 0.0

    def _tick(self):
        hazard = self._hazard()

        # Hold a block briefly even once the sector reads clear. Without this a
        # single optimistic scan releases the guard for one tick, which the car
        # feels as a lurch rather than as motion.
        if self.blocked and hazard is None:
            if time.time() - self.blocked_since < self.min_block_s:
                hazard = self.blocked

        if hazard != self.blocked:
            if hazard:
                self.blocked_since = time.time()
                verb = 'stopping' if self.enabled else 'IGNORED (guard overridden)'
                self.get_logger().warn(
                    f'obstacle {hazard} at {self.sectors[hazard]:.2f} m, {verb}')
            else:
                self.get_logger().info('clear')
            self.blocked = hazard

        # Say so, repeatedly, for as long as it is off. An override that goes
        # quiet is one that gets left on by accident.
        if not self.enabled:
            now = time.time()
            if now - self._last_warn >= self.warn_every:
                self._last_warn = now
                would = f', would be stopping for {hazard}' if hazard else ''
                self.get_logger().warn(f'LiDAR guard is OVERRIDDEN{would}')

        self.state_pub.publish(String(data=json.dumps({
            'enabled': self.enabled,
            'would_block': hazard,
            'blocked': hazard if self.enabled else None,
            'intent': round(self.intent, 3) if (
                time.time() - self.intent_at <= self.intent_timeout) else 0.0,
            'sectors': {k: (round(v, 2) if v < 99 else None)
                        for k, v in self.sectors.items()},
        })))

        # Publish ONLY while intervening. The arbiter ranks safety above
        # everything but the e-stop, so a continuous publisher would veto the
        # whole system permanently. Withholding this one publish is the entire
        # mechanism of the override: everything above still ran.
        if hazard and self.enabled:
            self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
