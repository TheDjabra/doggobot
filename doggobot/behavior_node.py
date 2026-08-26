#!/usr/bin/env python3
"""Behaviours: the vocabulary of things the car can be told to do.

Sole publisher of /behavior_cmd. Exactly one primitive runs at a time, which is
the whole point: "circle left" must cancel "forward" rather than blending with
it.

Accepts commands on /voice_cmd, in either shape, so speech and buttons converge:

    {"action": "circle_left"}          from a button, or later from the LLM tier
    {"text": "circle to the left"}     from speech, matched by keyword

Keyword matching lives here rather than in the phone, deliberately. Both input
paths (on-robot microphone and phone) publish to /voice_cmd, so putting the
vocabulary next to the primitives means neither path can drift out of sync with
what the car can actually do.

Primitives:

    stop            cancel everything, publish nothing more
    wait            hold stopped, stay the active primitive
    forward         straight ahead
    reverse         straight back
    circle_right    forward with steering held right
    circle_left     forward with steering held left
    follow          relay the follow controller's output

Every moving primitive is TIME-BOUNDED by default. "Forward" meaning "until
further notice" is how a car ends up in a wall when a network drops; a duration
makes every command self-limiting, and an explicit `{"seconds": N}` overrides it.
The same idea as fall-2024 Team 12 packing a timeout into their command messages.

`follow` is a primitive rather than a parallel system so that mutual exclusion is
structural: commanding anything else stops following, with no arbitration needed.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

# Longest match first, so "circle left" cannot be swallowed by "left".
#
# Note these include SINGLE-WORD forms as well as full phrases, because Vosk's
# constrained grammar is a WORD list rather than a phrase list: given the phrase
# "back up" in the vocabulary it will happily return just "back". Matching only
# full phrases meant a perfectly good recognition ("back") matched nothing.
KEYWORDS = [
    ('circle_right', ('circle right', 'circle to the right', 'turn circles right')),
    ('circle_left',  ('circle left', 'circle to the left', 'turn circles left')),
    ('reverse',      ('reverse', 'back up', 'go back', 'backward', 'backwards',
                      'back')),
    ('forward',      ('go forward', 'forward', 'go straight', 'straight',
                      'ahead')),
    ('follow',       ('follow me', 'follow', 'come here')),
    ('wait',         ('wait', 'hold', 'stay', 'freeze')),
    ('stop',         ('stop it', 'stop', 'halt')),
]


class BehaviorNode(Node):

    def __init__(self):
        super().__init__('behavior_node')

        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('cruise_throttle', 0.16)
        self.declare_parameter('reverse_throttle', 0.15)
        self.declare_parameter('circle_steer', 0.7)
        self.declare_parameter('default_seconds', 3.0)
        self.declare_parameter('circle_seconds', 6.0)
        self.declare_parameter('max_seconds', 15.0)

        g = self.get_parameter
        self.hz = float(g('publish_hz').value)
        self.cruise = float(g('cruise_throttle').value)
        self.rev = float(g('reverse_throttle').value)
        self.circle_steer = float(g('circle_steer').value)
        self.default_s = float(g('default_seconds').value)
        self.circle_s = float(g('circle_seconds').value)
        self.max_s = float(g('max_seconds').value)

        self.active = None          # primitive name, or None when idle
        self.until = 0.0            # wall-clock deadline; 0 means open-ended
        self.follow_cmd = Twist()
        self.follow_fresh = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'behavior_cmd', 10)
        self.lock_pub = self.create_publisher(String, 'target_lock', 10)
        self.state_pub = self.create_publisher(String, 'behavior_state', 10)

        self.create_subscription(String, 'voice_cmd', self._on_command, 10)
        self.create_subscription(Twist, 'follow_cmd', self._on_follow, 10)
        self.create_subscription(String, 'target_state', self._on_target, 10)

        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'behaviours ready: cruise {self.cruise}, circle steer '
            f'{self.circle_steer}, default {self.default_s:.0f}s')

    # -- inputs ---------------------------------------------------------------

    def _match(self, text):
        t = ' '.join(text.lower().split())
        for name, phrases in KEYWORDS:
            for p in phrases:
                if p in t:
                    return name
        return None

    def _on_command(self, msg):
        try:
            m = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return

        action = m.get('action')
        if not action:
            text = (m.get('text') or '').strip()
            if not text:
                return
            action = self._match(text)
            if not action:
                self.get_logger().info(f'no primitive matched: {text!r}')
                return
            self.get_logger().info(f'"{text}" -> {action}')

        seconds = m.get('seconds')
        self._start(action, float(seconds) if seconds else None)

    def _on_follow(self, msg):
        self.follow_cmd = msg
        self.follow_fresh = time.time()

    def _on_target(self, msg):
        """Enter and leave follow mode from perception's lock state.

        Following is driven by whether a target is locked, so the phone's
        existing FOLLOW button keeps working untouched, and a lock dropped by the
        tracker ends the primitive without anything else having to notice.
        """
        try:
            locked = bool(json.loads(msg.data).get('locked'))
        except Exception:                                    # noqa: BLE001
            return
        if locked and self.active != 'follow':
            self._enter('follow', 0.0)
        elif not locked and self.active == 'follow':
            self.get_logger().info('follow ended: lock lost')
            self._enter(None, 0.0)

    # -- primitive control ----------------------------------------------------

    def _enter(self, name, until):
        self.active, self.until = name, until
        self.state_pub.publish(String(data=json.dumps({
            'active': name, 'until': until, 'stamp': time.time()})))

    def _start(self, action, seconds=None):
        if action in ('stop', 'release'):
            if self.active == 'follow':
                self._release_lock()
            self.get_logger().info('stop')
            self._enter(None, 0.0)
            self.cmd_pub.publish(Twist())
            return

        if action == 'follow':
            # perception's lock is the source of truth; asking for it is enough.
            self.lock_pub.publish(String(data=json.dumps({'action': 'lock'})))
            return

        # Any other primitive cancels following, including its lock, so the car
        # is not still chasing someone while driving a canned trajectory.
        if self.active == 'follow':
            self._release_lock()

        if seconds is None:
            seconds = self.circle_s if action.startswith('circle') else self.default_s
        seconds = max(0.5, min(self.max_s, seconds))

        if action not in ('wait', 'forward', 'reverse', 'circle_right', 'circle_left'):
            self.get_logger().warn(f'unknown primitive: {action}')
            return

        self.get_logger().info(f'{action} for {seconds:.1f}s')
        self._enter(action, time.time() + seconds)

    def _release_lock(self):
        self.lock_pub.publish(String(data=json.dumps({'action': 'release'})))

    # -- output ---------------------------------------------------------------

    def _tick(self):
        if self.active is None:
            return                      # silent: the arbiter times out and stops

        if self.active == 'follow':
            # Relay only while the controller is actually producing commands.
            if time.time() - self.follow_fresh < 0.5:
                self.cmd_pub.publish(self.follow_cmd)
            return

        if self.until and time.time() >= self.until:
            self.get_logger().info(f'{self.active} finished')
            self._enter(None, 0.0)
            self.cmd_pub.publish(Twist())
            return

        cmd = Twist()
        if self.active == 'forward':
            cmd.linear.x = self.cruise
        elif self.active == 'reverse':
            cmd.linear.x = -self.rev
        elif self.active == 'circle_right':
            cmd.linear.x, cmd.angular.z = self.cruise, self.circle_steer
        elif self.active == 'circle_left':
            cmd.linear.x, cmd.angular.z = self.cruise, -self.circle_steer
        # 'wait' falls through as a zero Twist: stopped, but still the active
        # primitive, so it is a held state rather than an absence of one.
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
