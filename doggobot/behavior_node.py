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

Also runs SEQUENCES, either spoken as a chain:

    "forward then circle left then stop"

or given structurally, which is what the LLM tier will emit:

    {"action": "sequence", "steps": [
        {"action": "forward", "seconds": 2},
        {"action": "forward", "metres": 1.5},
        {"action": "circle_right", "degrees": 30},
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
    turn_around     roughly 180 degrees, as a held circle. The mission statement
                    asks the car to "turn around" and there was no way to say it:
                    a circle is not a turn-around to anyone watching.
    three_point     forward on full lock, reverse on opposite lock, forward. Turns
                    the car around in FAR less space than a circle, which matters
                    indoors where the car's Ackermann turning radius is the real
                    constraint. Expands into a sequence, reusing the executor.
    rev_left        reverse while steering left    ) the pieces three_point is
    rev_right       reverse while steering right   ) built from
    figure_eight    a circle each way, expanded into a two-step sequence rather
                    than special-cased, so it reuses the executor it would
                    otherwise duplicate
    follow          relay the follow controller's output
    color_react     TEST MODE: drive forward while green is seen, reverse while
                    red is seen, stop otherwise. Continuous rather than
                    sequenced, for tuning thresholds with the wheels up and
                    without speaking.

Every moving primitive is TIME-BOUNDED by default. "Forward" meaning "until
further notice" is how a car ends up in a wall when a network drops; a duration
makes every command self-limiting, and an explicit `{"seconds": N}` overrides it.
The same idea as fall-2024 Team 12 packing a timeout into their command messages.

`follow` is a primitive rather than a parallel system so that mutual exclusion is
structural: commanding anything else stops following, with no arbitration needed.
"""
import json
import re
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

# Longest match first, so "circle left" cannot be swallowed by "left".
#
# Note these include SINGLE-WORD forms as well as full phrases, because Vosk's
# constrained grammar is a WORD list rather than a phrase list: given the phrase
# "back up" in the vocabulary it will happily return just "back". Matching only
# full phrases meant a perfectly good recognition ("back") matched nothing.
KEYWORDS = [
    # LOOK FIRST, and this order is load-bearing. Matching is first-wins on a
    # substring, and "look forward" contains "forward", so with the drive
    # primitives ahead of these a request to turn the CAMERA drove the CAR.
    # Caught on the vehicle 2026-09-01. Anything added below must not be a
    # substring of a look phrase.
    ('look_left',    ('look left', 'look to the left', 'camera left')),
    ('look_right',   ('look right', 'look to the right', 'camera right')),
    ('look_forward', ('look forward', 'look ahead', 'look straight',
                      'eyes front', 'camera centre', 'camera center')),
    ('figure_eight', ('figure eight', 'figure of eight', 'figure 8', 'do a figure eight')),
    ('three_point',  ('three point turn', 'three point', '3 point turn',
                      'k turn', 'point turn')),
    ('turn_around',  ('turn around', 'turn round', 'about face', 'come back',
                      'go back the other way', 'u turn', 'reverse direction')),
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


# Matches pan_node.PAN_CEILING_DEG and the firmware's ABS_LIMIT_DEG. Duplicated
# rather than imported so each layer holds the bound on its own.
PAN_CEILING_DEG = 90.0


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
        # The colour test mode is exempt from the short default because it is
        # meant to be watched while tuning, but it is still bounded: an
        # open-ended reactive mode is exactly the thing that drives off a bench.
        self.declare_parameter('color_react_seconds', 300.0)
        # Autonomy suppression. When the operator is on the manual DRIVE tab,
        # nothing autonomous may move the car: no primitive, no sequence, no
        # follow. This is a mode guarantee, not a preference, and it is the same
        # shape as the arm gate: separate what the car CAN do from what someone
        # currently intends.
        self.declare_parameter('autonomy_enabled', True)
        # How long a held full-lock circle takes to come back around. This is
        # geometry and surface dependent (steering throw, wheelbase, grip), so it
        # is a measured number rather than a computed one: drive it, watch where
        # it ends up, adjust.
        self.declare_parameter('turn_around_seconds', 3.0)
        # Per-leg time for the three-point turn. All three legs are geometry and
        # grip dependent, so these are measured, not computed: drive it, see
        # where the nose ends up, adjust.
        self.declare_parameter('three_point_forward_s', 1.6)
        self.declare_parameter('three_point_reverse_s', 1.6)
        self.declare_parameter('three_point_settle_s', 0.8)

        # Dead reckoning. A step may ask for METRES or DEGREES instead of
        # seconds, and these two numbers convert them.
        #
        # Open-loop by necessity: the VESC reports a tachometer, but the class
        # `vesc_twist_node` holds the serial port exclusively and publishes no
        # telemetry, so nothing can read wheel travel while the stack runs.
        # Closing that loop means replacing the class actuator node.
        #
        # CALIBRATE BOTH, they are not computed:
        #   metres: command `forward` for a known time, measure with a tape,
        #           divide. Changes with surface, slope and pack charge.
        #   degrees: command `turn_around`, see how far the nose actually swung,
        #           divide by the time.
        self.declare_parameter('metres_per_second', 0.9)
        self.declare_parameter('degrees_per_second', 60.0)
        self.declare_parameter('max_metres', 8.0)

        # The on-robot microphone listens continuously and the vocabulary is made
        # of very common English words (stop, back, forward, wait, left, right).
        # Without a gate, anyone talking near the car can drive it: observed
        # 2026-08-26, when overheard speech matched "back" and reversed the car
        # mid-test. The phone does not need this because holding the talk button
        # is already a deliberate act, so the requirement is per-source.
        # -- camera pan axis --
        # behavior_node is the sole /pan_cmd publisher for the same reason it is
        # the sole /behavior_cmd publisher: follow tracking, a spoken "look left"
        # and an idle recentre are three claimants on one actuator.
        self.declare_parameter('pan_enabled', True)
        self.declare_parameter('pan_look_deg', 45.0)
        self.declare_parameter('pan_centre_on_idle', True)
        self.declare_parameter('pan_publish_hz', 20.0)
        # Escalation: if the camera sits pinned far off the nose, the chassis has
        # failed to turn to face the target and steering harder will not fix it,
        # because a car cannot turn in place. A three-point turn can.
        # OFF by default: it is the newest and least tested path here, and an
        # unexpected three-point turn during the demo is worse than a wide arc.
        self.declare_parameter('pan_escalate', False)
        self.declare_parameter('pan_escalate_deg', 55.0)
        self.declare_parameter('pan_escalate_s', 2.5)

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
        self.color_react_s = float(g('color_react_seconds').value)
        self.autonomy = bool(g('autonomy_enabled').value)
        self.turn_around_s = float(g('turn_around_seconds').value)
        self.tp_fwd_s = float(g('three_point_forward_s').value)
        self.tp_rev_s = float(g('three_point_reverse_s').value)
        self.tp_settle_s = float(g('three_point_settle_s').value)
        self.mps = max(0.05, float(g('metres_per_second').value))
        self.dps = max(1.0, float(g('degrees_per_second').value))
        self.max_m = float(g('max_metres').value)
        self.pan_enabled = bool(g('pan_enabled').value)
        self.pan_look = float(g('pan_look_deg').value)
        self.pan_centre_idle = bool(g('pan_centre_on_idle').value)
        self.pan_escalate = bool(g('pan_escalate').value)
        self.pan_escalate_deg = float(g('pan_escalate_deg').value)
        self.pan_escalate_s = float(g('pan_escalate_s').value)

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

        self.follow_pan = 0.0       # angle the follow cascade wants
        self.follow_pan_fresh = 0.0
        self.pan_manual = None      # angle a spoken "look left" is holding
        self.pan_deg = None         # measured, from pan_node
        self.pan_over_since = 0.0   # when the pan angle first pinned out
        self.escalating = False     # a manoeuvre WE started; do not cancel it

        self.cmd_pub = self.create_publisher(Twist, 'behavior_cmd', 10)
        self.lock_pub = self.create_publisher(String, 'target_lock', 10)
        self.state_pub = self.create_publisher(String, 'behavior_state', 10)
        # Utterances the keyword parser could not handle. llm_node picks these up
        # and may publish a structured command back to /voice_cmd. Escalation is
        # a separate node on purpose: this one runs the control loop, and a
        # blocking network call inside it would stall the primitives.
        self.unparsed_pub = self.create_publisher(String, 'voice_unparsed', 10)

        self.create_subscription(String, 'voice_cmd', self._on_command, 10)
        self.create_subscription(Twist, 'follow_cmd', self._on_follow, 10)
        self.create_subscription(String, 'target_state', self._on_target, 10)
        self.create_subscription(String, 'condition_state', self._on_condition, 10)
        self.create_subscription(Bool, 'autonomy_enabled', self._on_autonomy, 10)
        self.pan_pub = self.create_publisher(Float32, 'pan_cmd', 10)
        self.create_subscription(Float32, 'follow_pan', self._on_follow_pan, 10)
        self.create_subscription(String, 'pan_state', self._on_pan_state, 10)
        # The phone's pan slider. Same standing as a spoken "look left": a manual
        # preference that yields to the follow cascade when a target is locked.
        self.create_subscription(Float32, 'pan_manual', self._on_pan_manual, 10)
        if self.pan_enabled:
            self.create_timer(1.0 / float(g('pan_publish_hz').value),
                              self._pan_tick)

        self.create_timer(1.0 / self.hz, self._tick)
        self.get_logger().info(
            f'behaviours ready: cruise {self.cruise}, circle steer '
            f'{self.circle_steer}, default {self.default_s:.0f}s')

    # -- inputs ---------------------------------------------------------------

    def _split_chain(self, text):
        """Split spoken chains into steps: "forward then circle left then stop".

        A keyword split, not language understanding. It covers the common case
        without the latency or the network dependency of an LLM, and the LLM tier
        can still handle anything with structure this cannot express (counts,
        durations, conditions).
        """
        t = ' '.join(text.lower().split())
        for sep in (' and then ', ' then ', ' after that '):
            t = t.replace(sep, ' | ')
        parts = [p.strip() for p in t.split('|')]
        return [p for p in parts if p]

    # "forward until green", "go forward till you see red"
    UNTIL_RE = re.compile(
        r'^(?P<action>.+?)\s+(?:until|till|til)\s+(?:you\s+see\s+|there\s+is\s+)?'
        r'(?:the\s+)?(?P<color>green|red)\b')
    # Anything conditional-looking at all, so an unrecognised condition can be
    # refused rather than silently dropped.
    CONDITIONAL_RE = re.compile(r'\b(?:until|till|til)\b')

    def _parse_step(self, text):
        """One spoken step, with an optional colour condition.

        Returns a step dict or None. Splitting this out is what lets a chain and
        a single command share the same grammar: "forward until green" works on
        its own and as a link in "forward until green then circle left".
        """
        text = text.strip()
        m = self.UNTIL_RE.match(text)
        if m:
            action = self._match(m.group('action'))
            if action:
                return {'action': action, 'until': {'color': m.group('color')}}
            return None

        # It asked for a condition and we could not parse the condition. Refusing
        # is the only safe answer: dropping the condition turns "drive until you
        # see green" into "drive for three seconds", which is a different and
        # much worse instruction than the one that was given.
        if self.CONDITIONAL_RE.search(text):
            return None

        action = self._match(text)
        return {'action': action} if action else None

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
            # A spoken chain becomes a sequence. Try this before single-command
            # matching, since "forward then stop" would otherwise match "forward"
            # and silently drop the rest.
            for text in candidates:
                parts = self._split_chain(text)
                if len(parts) < 2:
                    continue
                steps = [self._parse_step(p) for p in parts]
                if all(steps):
                    self.get_logger().info(
                        f'"{text}" -> sequence of {len(steps)}')
                    self._start_sequence(steps)
                    return
                # It was a chain and part of it did not parse. Do NOT fall
                # through to single-command matching, which would execute a
                # FRAGMENT of what was asked for: "forward then jump then stop"
                # becoming a plain forward is worse than nothing.
                #
                # But this is precisely the shape the LLM tier exists for
                # ("spin right two times then reverse"), so hand it over rather
                # than dropping it. If nothing is listening, behaviour is
                # unchanged from before the LLM existed.
                missed = [p for p, st in zip(parts, steps) if not st]
                self.get_logger().info(
                    f'chain not in vocabulary {missed}, escalating: "{text}"')
                self.unparsed_pub.publish(String(data=json.dumps(
                    {'text': text, 'source': source})))
                return

            # A single step may still carry a condition: "forward until green".
            for i, text in enumerate(candidates):
                step = self._parse_step(text)
                if step is None and self.CONDITIONAL_RE.search(text):
                    # Same reasoning: never approximate a condition, but a
                    # condition we cannot parse is worth one look from the LLM.
                    self.get_logger().info(
                        f'condition not in vocabulary, escalating: "{text}"')
                    self.unparsed_pub.publish(String(data=json.dumps(
                        {'text': text, 'source': source})))
                    return
                if step:
                    note = '' if i == 0 else f' (alternative {i})'
                    if step.get('until'):
                        self.get_logger().info(
                            f'"{text}" -> {step["action"]} until '
                            f'{step["until"]}{note}')
                        self._start_sequence([step])
                        return
                    action = step['action']
                    self.get_logger().info(f'"{text}" -> {action}{note}')
                    break
            if not action:
                self.get_logger().info(
                    f'no primitive matched: {candidates[0]!r}'
                    + (f' (+{len(candidates) - 1} alternatives)'
                       if len(candidates) > 1 else ''))
                # Hand it to the slow path. If nothing is listening, this is a
                # message into the void and the system behaves exactly as before.
                self.unparsed_pub.publish(String(data=json.dumps(
                    {'text': candidates[0], 'source': source})))
                return

        if not self.autonomy and action not in ('stop', 'release'):
            # Stop always works, whatever the mode. Refusing to stop would be an
            # absurd way to enforce a safety mode.
            self.get_logger().info(f'ignored ({action}): autonomy suppressed')
            return

        if action == 'sequence':
            self._start_sequence(m.get('steps'))
            return

        seconds = m.get('seconds')
        self._start(action, float(seconds) if seconds else None,
                    metres=m.get('metres'), degrees=m.get('degrees'))

    def _on_follow(self, msg):
        self.follow_cmd = msg
        self.follow_fresh = time.time()

    def _on_follow_pan(self, msg):
        self.follow_pan = float(msg.data)
        self.follow_pan_fresh = time.time()

    def _on_pan_manual(self, msg):
        self.pan_manual = max(-PAN_CEILING_DEG,
                              min(PAN_CEILING_DEG, float(msg.data)))

    def _on_pan_state(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return
        self.pan_deg = d.get('deg') if d.get('ok') else None

    def _pan_tick(self):
        """Sole writer to /pan_cmd. Priority: follow > manual look > centre.

        Following outranks a spoken "look left" because the tracker is closing a
        loop and the look is a one-off preference; a manual angle held during a
        follow would just be fought, once per frame, by a controller that has
        better information about where the target is.
        """
        now = time.time()
        following = (self.active == 'follow'
                     and (now - self.follow_pan_fresh) < 0.5)

        if following:
            if self.pan_manual is not None:
                self.pan_manual = None      # the tracker has the camera now
            target = self.follow_pan
        elif self.pan_manual is not None:
            target = self.pan_manual
        elif self.pan_centre_idle:
            target = 0.0
        else:
            return

        self.pan_pub.publish(Float32(data=float(target)))
        self._check_escalation(following, now)

    def _check_escalation(self, following, now):
        """A camera pinned hard off the nose means the chassis has given up.

        Steering is already saturated at that point, so the outer loop has no
        authority left; what it needs is a manoeuvre, not more gain.
        """
        if not (self.pan_escalate and following) or self.pan_deg is None:
            self.pan_over_since = 0.0
            return
        if abs(self.pan_deg) <= self.pan_escalate_deg:
            self.pan_over_since = 0.0
            return
        if self.pan_over_since == 0.0:
            self.pan_over_since = now
            return
        if now - self.pan_over_since < self.pan_escalate_s:
            return

        self.pan_over_since = 0.0
        self.get_logger().info(
            f'pan pinned at {self.pan_deg:+.0f} deg for {self.pan_escalate_s:g}s: '
            'the chassis cannot come round, escalating to a three point turn')
        # The lock is deliberately kept. This is a manoeuvre in service of the
        # follow, not an abandonment of it, and the flag stops _on_target from
        # immediately cancelling the very sequence we just started.
        self.escalating = True
        self._start('three_point')

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
        if locked and not self.autonomy:
            # A lock acquired while suppressed must not start driving.
            self._release_lock()
            return
        if locked and self.active != 'follow':
            if self.escalating:
                return          # a turn we started on purpose; let it finish
            self._cancel_sequence('follow lock acquired')
            self._enter('follow', 0.0)
        elif not locked and self.active == 'follow':
            self.get_logger().info('follow ended: lock lost')
            self._enter(None, 0.0)

    def _on_autonomy(self, msg):
        """Enable or suppress everything autonomous.

        Suppression CANCELS rather than warns. A mode that silently lets a
        sequence keep running is worse than no mode: the whole point is being
        able to look at the screen and know the car will not move on its own.
        """
        want = bool(msg.data)
        if want == self.autonomy:
            return
        self.autonomy = want
        if want:
            self.get_logger().info('autonomy enabled')
            return
        self.get_logger().warn('autonomy suppressed (manual drive)')
        self._cancel_sequence('autonomy suppressed')
        if self.active == 'follow':
            self._release_lock()
        if self.active is not None:
            self._enter(None, 0.0)
            self.cmd_pub.publish(Twist())

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
            'autonomy': self.autonomy,
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
            self.escalating = False
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
        self._start(step.get('action'), step.get('seconds'), in_sequence=True,
                    metres=step.get('metres'), degrees=step.get('degrees'))

    def _cancel_sequence(self, why):
        if self.sequence or self.seq_len:
            self.get_logger().info(f'sequence cancelled: {why}')
        self.escalating = False
        self.sequence = []
        self.seq_len = 0
        self.step_condition = None

    def _duration_for(self, action, seconds, metres, degrees):
        """Turn a distance or an angle into a time, using calibrated rates."""
        if seconds is not None:
            return float(seconds)
        if metres is not None:
            m = max(0.0, min(self.max_m, abs(float(metres))))
            secs = m / self.mps
            self.get_logger().info(
                f'{m:g} m at {self.mps:g} m/s -> {secs:.1f}s')
            return secs
        if degrees is not None:
            d = max(0.0, min(360.0, abs(float(degrees))))
            secs = d / self.dps
            self.get_logger().info(
                f'{d:g} deg at {self.dps:g} deg/s -> {secs:.1f}s')
            return secs
        return None

    def _start(self, action, seconds=None, in_sequence=False,
               metres=None, degrees=None):
        # The look primitives move a DIFFERENT actuator, so they are handled
        # before the drive-axis bookkeeping below. "Look left" while driving
        # forward should turn the camera and nothing else: it must not cancel a
        # running sequence, and it must not stop the car.
        if action in ('look_left', 'look_right', 'look_forward'):
            if not self.pan_enabled:
                self.get_logger().warn('no pan axis configured')
                return
            if action == 'look_forward':
                self.pan_manual = 0.0
            else:
                mag = abs(float(degrees)) if degrees is not None else self.pan_look
                mag = max(0.0, min(PAN_CEILING_DEG, mag))
                self.pan_manual = mag if action == 'look_right' else -mag
            if self.active == 'follow':
                self.get_logger().info(
                    'camera is tracking a target; the look will apply once the '
                    'follow ends')
            else:
                self.get_logger().info(f'look: pan to {self.pan_manual:+.0f} deg')
            return

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

        # A figure-eight is just one circle each way. Expanding it into a
        # sequence rather than adding a bespoke primitive means it inherits the
        # executor's cancellation, timeouts and reporting for free.
        # A three-point turn is a sequence, not a bespoke primitive, so it
        # inherits cancellation, timeouts and progress reporting for free.
        # Nose swings right, back up while swinging the tail the other way, then
        # straighten out facing the way it came.
        if action == 'three_point':
            self.get_logger().info(
                f'three point turn ({self.tp_fwd_s:g}/{self.tp_rev_s:g}/'
                f'{self.tp_settle_s:g}s)')
            self._start_sequence([
                {'action': 'circle_right', 'seconds': self.tp_fwd_s},
                {'action': 'rev_left', 'seconds': self.tp_rev_s},
                {'action': 'circle_right', 'seconds': self.tp_settle_s},
            ])
            return

        if action == 'figure_eight':
            half = seconds if seconds else self.circle_s
            self.get_logger().info(f'figure eight, {half:.0f}s each way')
            self._start_sequence([
                {'action': 'circle_left', 'seconds': half},
                {'action': 'circle_right', 'seconds': half},
            ])
            return

        if action == 'follow':
            # perception's lock is the source of truth; asking for it is enough.
            self.lock_pub.publish(String(data=json.dumps({'action': 'lock'})))
            return

        # Any other primitive cancels following, including its lock, so the car
        # is not still chasing someone while driving a canned trajectory.
        if self.active == 'follow':
            self._release_lock()

        seconds = self._duration_for(action, seconds, metres, degrees)

        if seconds is None:
            if action == 'turn_around':
                seconds = self.turn_around_s
            elif action == 'color_react':
                seconds = self.color_react_s
            elif action.startswith('circle'):
                seconds = self.circle_s
            else:
                seconds = self.default_s
        cap = self.color_react_s if action == 'color_react' else self.max_s
        seconds = max(0.5, min(cap, seconds))

        if action not in ('wait', 'forward', 'reverse', 'circle_right',
                          'circle_left', 'turn_around', 'rev_left', 'rev_right',
                          'color_react'):
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

        if self.active == 'color_react':
            # Reactive, not sequenced: read the latest colour every tick.
            seen = self.conditions.get('color')
            cmd = Twist()
            if seen == 'green':
                cmd.linear.x = self.cruise
            elif seen == 'red':
                cmd.linear.x = -self.rev
            self.cmd_pub.publish(cmd)
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
        elif self.active == 'rev_left':
            cmd.linear.x, cmd.angular.z = -self.rev, -self.circle_steer
        elif self.active == 'rev_right':
            cmd.linear.x, cmd.angular.z = -self.rev, self.circle_steer
        elif self.active == 'turn_around':
            # Full lock one way for a measured time. Not a three-point turn: this
            # car has no reverse-steer sequencing, and a tight sustained circle
            # gets the nose pointing back the way it came, which is what the
            # mission statement needs.
            cmd.linear.x, cmd.angular.z = self.cruise, self.circle_steer
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
