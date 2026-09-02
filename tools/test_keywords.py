#!/usr/bin/env python3
"""Keyword matching: does each spoken phrase reach the primitive it names?

_match is first-wins over a SUBSTRING, so the order of KEYWORDS is behaviour,
not presentation. "look forward" contains "forward", and with the drive
primitives listed first that phrase matched `forward` and drove the car when it
was asked to turn the camera. Found on the vehicle 2026-09-01.

Nothing about that failure is visible by reading either list on its own, and it
is silent: a real primitive runs, just not the one that was asked for. Hence a
test whose whole job is the collisions.

Run: python3 tools/test_keywords.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rosstub                                               # noqa: E402
rosstub.install()
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from doggobot.behavior_node import BehaviorNode, KEYWORDS     # noqa: E402

CASES = [
    # the collision that actually happened
    ('look forward',                 'look_forward'),
    ('atlas look forward',           'look_forward'),
    ('look ahead',                   'look_forward'),
    ('look straight',                'look_forward'),
    # camera, not wheels
    ('look left',                    'look_left'),
    ('look right',                   'look_right'),
    ('look to the left',             'look_left'),
    ('camera centre',                'look_forward'),
    # wheels, not camera
    ('go forward',                   'forward'),
    ('forward',                      'forward'),
    ('go straight',                  'forward'),
    ('reverse',                      'reverse'),
    ('back up',                      'reverse'),
    ('circle left',                  'circle_left'),
    ('circle right',                 'circle_right'),
    ('turn around',                  'turn_around'),
    ('three point turn',             'three_point'),
    ('figure eight',                 'figure_eight'),
    ('follow me',                    'follow'),
    ('stop',                         'stop'),
    ('wait',                         'wait'),
]

node = BehaviorNode()
ok = 0
print('phrase -> primitive\n')
for text, want in CASES:
    got = node._match(text)
    good = got == want
    ok += good
    print(f'  [{"OK  " if good else "WRONG"}] {text!r:<28} -> {got}'
          + ('' if good else f'   want {want}'))

# Every phrase must reach its own primitive. A phrase that is a substring of an
# EARLIER entry's phrase can never win, and that is exactly the failure above.
print('\nshadowed phrases (unreachable because an earlier entry swallows them):')
shadow = 0
for i, (name, phrases) in enumerate(KEYWORDS):
    for ph in phrases:
        hit = node._match(ph)
        if hit != name:
            shadow += 1
            print(f'  {ph!r} belongs to {name} but matches {hit}')
if not shadow:
    print('  none')

# ---------------------------------------------------------------------------
# Quantities. Parsing one and USING it are separate failures, and the second is
# what actually bit: "forward 5 m" matched the keyword, dropped the 5 m, and ran
# the default three seconds. The duration below is the real check.
# ---------------------------------------------------------------------------
import time                                                   # noqa: E402
from doggobot.behavior_node import parse_quantity              # noqa: E402

print('\nquantity extraction')
QCASES = [
    ('forward 5 m',                   {'metres': 5.0}),
    ('reverse 10 cm',                 {'metres': 0.1}),
    ('forward 2 meters',              {'metres': 2.0}),
    ('go right at a 30 degree angle', {'degrees': 30.0}),
    ('forward 3 seconds',             {'seconds': 3.0}),
    ('reverse 2 feet',                {'metres': 0.6096}),
    # A bare number is NOT a quantity, or these would be distances.
    ('figure 8',                      {}),
    ('3 point turn',                  {}),
    ('circle left',                   {}),
    # Beyond the regex on purpose, so it still escalates to the LLM.
    ('reverse half a foot',           {}),
]
qok = 0
for text, want in QCASES:
    got = parse_quantity(text)
    good = all(abs(got.get(k, -9) - v) < 1e-6 for k, v in want.items()) and \
        set(got) == set(want)
    qok += good
    print(f'  [{"OK  " if good else "WRONG"}] {text!r:32} -> {got}'
          + ('' if good else f'  want {want}'))

print('\nend to end: does the duration reflect the distance?')
rosstub.OVERRIDES = {'autonomy_enabled': True, 'metres_per_second': 0.635,
                     'coast_metres': 0.144, 'degrees_per_second': 45.0}
import doggobot.behavior_node as bn                            # noqa: E402
n2 = bn.BehaviorNode()
eok = 0
ECASES = [
    ('forward 5 m',                   (5.0 - 0.144) / 0.635),
    ('reverse 2 m',                   (2.0 - 0.144) / 0.635),
    ('circle right 30 degrees',       30.0 / 45.0),
    ('forward',                       3.0),      # no quantity: the default
    # NOTE "go right at a 30 degree angle" is deliberately NOT here: the offline
    # vocabulary has no "go right", only "circle right", so that phrasing
    # escalates to the LLM. Worth knowing, because it means the angle commands
    # people say most naturally depend on the LLM host being reachable.
]
for text, want_s in ECASES:
    n2.sent.clear()
    n2._on_command(rosstub.Data(json.dumps({'text': text, 'source': 'test'})))
    got_s = (n2.until - time.time()) if n2.until else 0.0
    good = abs(got_s - want_s) < 0.15
    eok += good
    print(f'  [{"OK  " if good else "WRONG"}] {text!r:32} -> {got_s:5.2f}s'
          f'   want {want_s:5.2f}s')

print(f'\n{ok}/{len(CASES)} phrases correct, {shadow} shadowed, '
      f'{qok}/{len(QCASES)} quantities, {eok}/{len(ECASES)} durations')
results_ok = (ok == len(CASES) and shadow == 0
              and qok == len(QCASES) and eok == len(ECASES))
sys.exit(0 if results_ok else 1)
