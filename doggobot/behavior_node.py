#!/usr/bin/env python3
"""Behaviours: the vocabulary of things the car can be told to do.

Sole publisher of /behavior_cmd. Exactly one primitive runs at a time, which is
the whole point: "circle left" must cancel "forward" rather than blending with
it.

Commands from the on-robot microphone must be prefixed with the wake word
("atlas forward"), because that mic listens continuously and the vocabulary is
ordinary English. Phone speech does not need it: holding the talk button is the
gate. See `wake_word_sources`.

Accepts commands on /voice_cmd, in either shape, so speech and buttons converge:

    {"action": "circle_left"}          from a button, or later from the LLM tier
    {"text": "circle to the left"}     from speech, matched by keyword

Keyword matching lives here rather than in the phone, deliberately. Both input
paths (on-robot microphone and phone) publish to /voice_cmd, so putting the
vocabulary next to the primitives means neither path can drift out of sync with
what the car can actually do.

Also runs SEQUENCES: an ordered list of steps executed one at a time.

    {"action": "sequence", "steps": [
        {"action": "forward", "seconds": 2},
        {"action": "circle_left"},
        {"action": "forward", "until": {"color": "green"}},
        {"action": "stop"}]}

A step ends when its duration lapses or its `until` condition is met, whichever
comes first. The duration therefore doubles as a timeout on every condition,
which matters: a sequence waiting forever on a green marker it will never see is
a car stuck in the middle of a demo with no way out.

Conditions are read from /condition_state, published by whatever can observe
them. Nothing publishes it yet, so a step with `until` currently falls back to
its duration and says so. That is deliberate: the executor is finished and the
sensor is not, and wiring colour detection in later is a subscriber rather than a
rewrite.

Any single command arriving mid-sequence cancels the whole sequence. Saying
"stop" must stop the car, not the current step of something that then continues.

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

        # The on-robot microphone listens continuously and the vocabulary is made
        # of very common English words (stop, back, forward, wait, left, right).
        # Without a gate, anyone talking near the car can drive it: observed
        # 2026-08-26, when overheard speech matched "back" and reversed the car
        # mid-test. The phone does not need this because holding the talk button
        # is already a deliberate act, so the requirement is per-source.
        self.declare_parameter('wake_word', 'atlas')
        self.declare_parameter('wake_word_sources', ['onboard-mic'])

        g = self.get_parameter
        self.hz = float(g('publish_hz').value)
        self.cruise = float(g('cruise_throttle').value)
        self.rev = float(g('reverse_throttle').value)
        self.circle_steer = float(g('circle_steer').value)
        self.default_s = float(g('default_seconds').value)
        self.circle_s = float(g('circle_seconds').value)
        self.max_s = float(g('max_seconds').value)
        self.wake = str(g('wake_word').value).lower().strip()
        self.wake_sources = set(g('wake_word_sources').value or [])

        self.active = None          # primitive name, or None when idle
        self.until = 0.0            # wall-clock deadline; 0 means open-ended
        self.sequence = []          # remaining steps
        self.seq_len = 0            # for reporting progress
        self.step_condition = None  # {"color": "green"} etc, or None
        self.conditions = {}        # latest observed conditions
        self.follow_cmd = Twist()
        self.follow_fresh = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'behavior_cmd', 10)
        self.lock_pub = self.create_publisher(String, 'target_lock', 10)
        self.state_pub = self.create_publisher(String, 'behavior_state', 10)

        self.create_subscription(String, 'voice_cmd', self._on_command, 10)
        self.create_subscription(Twist, 'follow_cmd', self._on_follow, 10)
        self.create_subscription(String, 'target_state', self._on_target, 10)
        self.create_subscription(String, 'condition_state', self._on_condition, 10)

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

    def _strip_wake(self, text):
        """Return the command after the wake word, or None if absent.

        Accepts the wake word anywhere in the utterance rather than only at the
        start, because recognisers frequently prepend filler.
        """
        words = text.lower().split()
        if self.wake not in words:
            return None
        return ' '.join(words[words.index(self.wake) + 1:])

    def _on_command(self, msg):
        try:
            m = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return

        source = m.get('source', '')
        needs_wake = self.wake and source in self.wake_sources

        action = m.get('action')
        if not action:
            # Try the top transcript, then any lower-ranked alternatives the
            # recogniser offered. A near-miss on the best guess is common in a
            # noisy room and the second guess is often exactly right.
            candidates = [(m.get('text') or '').strip()]
            candidates += [a.strip() for a in (m.get('alternatives') or [])]
            candidates = [c for c in candidates if c]
            if not candidates:
                return

            if needs_wake:
                gated = [self._strip_wake(c) for c in candidates]
                gated = [c for c in gated if c]
                if not gated:
                    self.get_logger().debug(
                        f'ignored (no wake word): {candidates[0]!r}')
                    return
                candidates = gated
            for i, text in enumerate(candidates):
                action = self._match(text)
                if action:
                    note = '' if i == 0 else f' (alternative {i})'
                    self.get_logger().info(f'"{text}" -> {action}{note}')
                    break
            if not action:
                self.get_logger().info(
                    f'no primitive matched: {candidates[0]!r}'
                    + (f' (+{len(candidates) - 1} alternatives)'
                       if len(candidates) > 1 else ''))
                return

        if action == 'sequence':
            self._start_sequence(m.get('steps'))
            return

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
            self._cancel_sequence('follow lock acquired')
            self._enter('follow', 0.0)
        elif not locked and self.active == 'follow':
            self.get_logger().info('follow ended: lock lost')
            self._enter(None, 0.0)

    def _on_condition(self, msg):
        """Latest observed world state, e.g. {"color": "green", "distance": 900}."""
        try:
            self.conditions = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def _condition_met(self):
        if not self.step_condition:
            return False
        for key, want in self.step_condition.items():
            if self.conditions.get(key) != want:
                return False
        return True

    # -- primitive control ----------------------------------------------------

    def _enter(self, name, until):
        self.active, self.until = name, until
        self.state_pub.publish(String(data=json.dumps({
            'active': name, 'until': until,
            'step': (self.seq_len - len(self.sequence)) if self.seq_len else 0,
            'steps': self.seq_len,
            'waiting_for': self.step_condition,
            'stamp': time.time()})))

    def _start_sequence(self, steps):
        if not isinstance(steps, list) or not steps:
            self.get_logger().warn('sequence with no steps')
            return
        steps = steps[:20]                     # a sane bound on a parsed command
        self.sequence = list(steps)
        self.seq_len = len(self.sequence)
        self.get_logger().info(
            f'sequence of {self.seq_len}: '
            + ' -> '.join(str(s.get('action')) for s in self.sequence))
        self._next_step()

    def _next_step(self):
        if not self.sequence:
            self.get_logger().info('sequence complete')
            self.seq_len = 0
            self.step_condition = None
            self._enter(None, 0.0)
            self.cmd_pub.publish(Twist())
            return
        step = self.sequence.pop(0)
        done = self.seq_len - len(self.sequence)
        self.get_logger().info(f'step {done}/{self.seq_len}: {step.get("action")}')
        self.step_condition = step.get('until')
        if self.step_condition and not self.conditions:
            self.get_logger().warn(
                f'step waits on {self.step_condition} but nothing publishes '
                '/condition_state yet; the duration will time it out')
        self._start(step.get('action'), step.get('seconds'), in_sequence=True)

    def _cancel_sequence(self, why):
        if self.sequence or self.seq_len:
            self.get_logger().info(f'sequence cancelled: {why}')
        self.sequence = []
        self.seq_len = 0
        self.step_condition = None

    def _start(self, action, seconds=None, in_sequence=False):
        if not in_sequence:
            # A single command mid-sequence cancels the whole thing. "Stop" must
            # stop the car, not just the current step of something continuing.
            self._cancel_sequence(f'{action} commanded')

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

        finished = self.until and time.time() >= self.until
        if self.step_condition and self._condition_met():
            self.get_logger().info(f'condition met: {self.step_condition}')
            finished = True

        if finished:
            if self.sequence or self.seq_len:
                self._next_step()
                return
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
