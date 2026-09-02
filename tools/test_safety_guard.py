#!/usr/bin/env python3
"""LiDAR guard: chatter, intervention distance, and the override.

Two claims to prove, and the second matters more than the first:

  1. An object sitting near the stop distance must not toggle the guard every
     tick. Measured on the car 2026-09-01: 16 stop/clear cycles in a minute,
     which a 3 second command feels as being chopped into fragments.

  2. Hysteresis must NOT delay intervention. It widens the distance at which a
     block is RELEASED, never the distance at which one starts, so the car must
     still stop at exactly the same range as before.

Run: python3 tools/test_safety_guard.py
"""
import json
import math
import os
import random
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rosstub                                               # noqa: E402
rosstub.install()
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

N_BEAMS = 360
TICK = 1 / 20.0            # publish_hz


def make_scan(right_m, far=10.0):
    """A scan that is empty except for a wedge on the car's right flank."""
    s = rosstub.LaserScan()
    s.angle_min = -math.pi
    s.angle_increment = 2 * math.pi / N_BEAMS
    s.range_min, s.range_max = 0.01, 12.0
    s.ranges = [far] * N_BEAMS
    for deg in range(-100, -80):            # right side, clear of front/rear
        s.ranges[int((math.radians(deg) - s.angle_min) / s.angle_increment)] = right_m
    return s


def build(**over):
    rosstub.OVERRIDES = dict(over)
    for m in list(sys.modules):
        if m.endswith('safety_node'):
            del sys.modules[m]
    from doggobot.safety_node import SafetyNode
    return SafetyNode()


def run(node, distances):
    """Feed a distance per tick; return (transitions, blocked_flags)."""
    moving = rosstub.Twist(); moving.linear.x = 0.3
    flags, trans, prev = [], 0, None
    for d in distances:
        node._on_intent(moving)
        node.intent_at = time.time()
        node._on_scan(make_scan(d))
        node._tick()
        b = node.blocked is not None
        if prev is not None and b != prev:
            trans += 1
        prev = b
        flags.append(b)
        time.sleep(TICK)
    return trans, flags


print('LiDAR guard: chatter and intervention distance\n')

random.seed(7)
# An object parked right on the 0.25 m side threshold, with realistic jitter.
dither = [0.25 + random.uniform(-0.03, 0.03) for _ in range(200)]   # 10 s

old = build(release_margin_m=0.0, min_block_s=0.0)
new = build()                                    # shipped defaults
t_old, f_old = run(old, dither)
t_new, f_new = run(new, dither)

print(f'  object dithering on the threshold, 10 s at 20 Hz')
print(f'    without hysteresis : {t_old:3d} stop/clear transitions, '
      f'blocked {100*sum(f_old)/len(f_old):5.1f}% of ticks')
print(f'    with hysteresis    : {t_new:3d} stop/clear transitions, '
      f'blocked {100*sum(f_new)/len(f_new):5.1f}% of ticks')
chatter_ok = t_new < t_old / 4

# Approach: does it still stop at the same range?
approach = [0.60 - 0.01 * i for i in range(45)]
stops = {}
for label, n in (('without', build(release_margin_m=0.0, min_block_s=0.0)),
                 ('with', build())):
    moving = rosstub.Twist(); moving.linear.x = 0.3
    for d in approach:
        n._on_intent(moving); n.intent_at = time.time()
        n._on_scan(make_scan(d)); n._tick()
        if n.blocked:
            stops[label] = d
            break
print(f'\n  approaching a wall, stop distance')
print(f'    without hysteresis : {stops.get("without"):.2f} m')
print(f'    with hysteresis    : {stops.get("with"):.2f} m')
same = abs(stops.get('without', -1) - stops.get('with', -2)) < 1e-9

print()
print(f'  [{"PASS" if chatter_ok else "FAIL"}] chatter cut by at least 4x')
print(f'  [{"PASS" if same else "FAIL"}] intervention distance unchanged')

# ---------------------------------------------------------------------------
# The override. On a stand the guard measures the bench and vetoes everything.
# ---------------------------------------------------------------------------

print('\n  override (guard off) behaviour')

moving = rosstub.Twist(); moving.linear.x = 0.3
BLOCKED = 0.10          # well inside the 0.25 m side threshold

def step(n, enabled=None):
    if enabled is not None:
        n._on_enable(rosstub.Data(enabled))
    n.sent.clear()
    n._on_intent(moving); n.intent_at = time.time()
    n._on_scan(make_scan(BLOCKED))
    n._tick()
    return n

g = build()
# positive control FIRST: if the veto never fires, "no veto when off" is
# worthless as evidence.
step(g)
vetoed_on = len(g.sent.get('safety_cmd', [])) > 0
default_on = g.enabled is True

step(g, enabled=False)
vetoed_off = len(g.sent.get('safety_cmd', [])) > 0
state_off = json.loads(g.sent['obstacle_state'][-1].data)

step(g, enabled=True)
vetoed_again = len(g.sent.get('safety_cmd', [])) > 0

print(f'    defaults to enabled            : {default_on}')
print(f'    obstacle at {BLOCKED} m, guard on  : veto published = {vetoed_on}')
print(f'    same obstacle, guard overridden: veto published = {vetoed_off}')
print(f'    re-enabled                     : veto published = {vetoed_again}')
print(f'    while overridden it still reports would_block = '
      f'{state_off.get("would_block")!r}, blocked = {state_off.get("blocked")!r}')

checks = [
    ('defaults to guard ON', default_on),
    ('vetoes when on (positive control)', vetoed_on),
    ('does NOT veto when overridden', not vetoed_off),
    ('vetoes again once re-enabled', vetoed_again),
    ('still reports what it would have stopped for',
     state_off.get('would_block') == 'right'),
    ('reports blocked=None while overridden', state_off.get('blocked') is None),
    ('reports enabled=False', state_off.get('enabled') is False),
]
for name, ok in checks:
    print(f'  [{"PASS" if ok else "FAIL"}] {name}')

allok = chatter_ok and same and all(o for _, o in checks)
print(f'\n{"ALL PASS" if allok else "FAILURES ABOVE"}')
sys.exit(0 if allok else 1)
