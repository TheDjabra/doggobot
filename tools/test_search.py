#!/usr/bin/env python3
"""Search sweep: does the camera actually go looking, and can it reacquire?

The sweep is the visible half and the easy half. The half that matters is the
lock request: perception clears want_lock when a target is REMOVED, so a camera
that sweeps beautifully and never asks for a lock again would find nobody, and
would look exactly like a camera that is working.

Uses a fake clock so the 2 second grace period and a 30 second timeout can be
checked in milliseconds, and drives the real BehaviorNode.

Run: python3 tools/test_search.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rosstub                                               # noqa: E402
rosstub.install()
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import doggobot.behavior_node as bn                          # noqa: E402


class Clock:
    def __init__(self):
        self.t = 1000.0
    def time(self):
        return self.t
    def advance(self, s):
        self.t += s


clock = Clock()
bn.time = clock

rosstub.OVERRIDES = {'search_enabled': True, 'search_delay_s': 2.0,
                     'search_range_deg': 45.0, 'search_speed_deg_s': 35.0,
                     'search_timeout_s': 30.0, 'pan_enabled': True,
                     'pan_centre_on_idle': True, 'autonomy_enabled': True}
node = bn.BehaviorNode()

TICK = 0.05
results = []


def check(name, ok, detail=''):
    results.append(ok)
    print(f'  [{"PASS" if ok else "FAIL"}] {name}' + (f'   {detail}' if detail else ''))


def target(locked, pan=None, status=None):
    if pan is not None:
        node._on_pan_state(rosstub.Data(json.dumps({'ok': True, 'deg': pan})))
    if status is None:
        status = 'TRACKED' if locked else 'NO_TARGET'
    node._on_target(rosstub.Data(json.dumps({'locked': locked,
                                             'status': status})))


LOCKS = []


def ticks(seconds):
    """Advance the fake clock in TICK steps, running the pan loop each step.

    Accumulates what was published across the whole span. Clearing per tick
    would throw away the one-shot lock request this test exists to observe,
    which is how the first version of this file reported a failure that was its
    own fault rather than the node's.
    """
    n = int(seconds / TICK)
    out = []
    for _ in range(n):
        clock.advance(TICK)
        node.sent.clear()
        node._pan_tick()
        for m in node.sent.get('pan_cmd', []):
            out.append(m.data)
        for m in node.sent.get('target_lock', []):
            LOCKS.append(json.loads(m.data)['action'])
    return out


def follow_again(pan=0.0):
    """Put the node back into a live follow, so the next loss is a real one."""
    target(True, pan=pan)
    node.follow_pan, node.follow_pan_fresh = pan, clock.time()
    node.active = 'follow'
    ticks(0.2)


print('search sweep\n')

node._on_arm(rosstub.Data(True))          # nothing sweeps while disarmed

# Acquire, then lose the target while the camera is pointing RIGHT.
target(True)
node.follow_pan, node.follow_pan_fresh = 20.0, clock.time()
node.active = 'follow'
ticks(0.2)
target(False, pan=20.0)
check('a lost lock is remembered', node.lost_at > 0)

# Grace period: nothing should move yet.
angles = ticks(1.5)
check('no sweep during the 2 s grace period',
      all(abs(a) < 1e-6 for a in angles) and not node.searching,
      f'{len(angles)} commands, all centred')

# Past the delay it should start, and ask for a lock.
LOCKS.clear()
angles = ticks(1.0)
check('sweep starts after the delay', node.searching)
check('RE-REQUESTS the lock, or it could never reacquire', 'lock' in LOCKS,
      f'published {LOCKS}')
check('sweeps toward the side it was lost on (right)',
      len(angles) > 2 and angles[-1] > angles[0],
      f'{angles[0]:+.1f} -> {angles[-1]:+.1f} deg')

# Long sweep: bounded, and it must reverse rather than stall at a limit.
angles = ticks(12.0)
check('stays inside +/-45 deg', max(abs(a) for a in angles) <= 45.0 + 1e-6,
      f'peak {max(abs(a) for a in angles):.1f} deg')
check('reverses at the limits', min(angles) < -40 and max(angles) > 40,
      f'{min(angles):+.1f} .. {max(angles):+.1f}')

# Reacquire cancels it.
target(True)
check('reacquiring a target ends the search', not node.searching)

# Manual look outranks the sweep.
follow_again(pan=-30.0)
target(False, pan=-30.0)
ticks(2.5)
check('sweep restarts toward the left after a left-side loss', node.searching)
node._on_pan_manual(rosstub.Data(45.0))
ticks(0.2)
check('a manual look ends the search', not node.searching)

# Timeout: give up, recentre, release the lock.
node.pan_manual = None
follow_again(pan=10.0)
target(False, pan=10.0)
ticks(2.5)
was = node.searching
LOCKS.clear()
angles = ticks(31.0)
check('was searching before the timeout', was)
check('gives up after the timeout', not node.searching)
check('releases the lock when giving up', 'release' in LOCKS,
      f'published {LOCKS}')
check('parks looking forward', abs(angles[-1]) < 1e-6, f'{angles[-1]:+.1f} deg')

# Disarm must stop the head, not just the wheels.
node._on_arm(rosstub.Data(True))
follow_again(pan=25.0)
target(False, pan=25.0)
ticks(2.5)
check('searching again before the disarm test', node.searching)
node.sent.clear()
node._on_arm(rosstub.Data(False))
check('DISARM ends the search', not node.searching)
parked = [m.data for m in node.sent.get('pan_cmd', [])]
check('disarm parks the camera forward', parked and abs(parked[-1]) < 1e-6,
      f'{parked}')
check('disarm gives the lock back',
      'release' in [json.loads(m.data)['action']
                    for m in node.sent.get('target_lock', [])])
angles = ticks(3.0)
check('stays parked while disarmed', all(abs(a) < 1e-6 for a in angles),
      f'{len(angles)} commands')

# A LOST tracklet is still LOCKED. The search clock must start when the target
# disappears, not when the tracker eventually gives up, or the 2 s delay would
# be stacked on top of the tracker's own timeout.
node._on_arm(rosstub.Data(True))
follow_again(pan=15.0)
target(True, pan=15.0, status='LOST')
check('a LOST tracklet starts the clock while still locked', node.lost_at > 0)
angles = ticks(1.5)
check('still no sweep inside the grace period', not node.searching)
ticks(1.0)
check('sweeps on sustained LOST without waiting for the lock to drop',
      node.searching)

# And if it simply reappears, that must cancel it rather than sweep anyway.
target(True, pan=15.0, status='TRACKED')
check('target reappearing cancels the search', not node.searching and not node.lost_at)

print(f'\n{sum(results)}/{len(results)} checks passed')
sys.exit(0 if all(results) else 1)
