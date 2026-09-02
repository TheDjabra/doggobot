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

print(f'\n{ok}/{len(CASES)} phrases correct, {shadow} shadowed')
sys.exit(0 if (ok == len(CASES) and shadow == 0) else 1)
