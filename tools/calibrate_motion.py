#!/usr/bin/env python3
"""Measure how far and how fast the car actually moves, and write it down.

behavior_node turns "go forward one metre" into a duration by dividing by
`metres_per_second`, and turns "30 degrees" into one by dividing by
`degrees_per_second`. Both of those are currently guesses, which means every
distance and angle command is confidently wrong. This settles them.

Two things this does that doing it by hand does not:

* It WATCHES THE GUARD during each run. If the LiDAR vetoes mid-run the car
  stops early, the tape says something shorter, and the resulting calibration is
  quietly wrong in a way nothing later would explain. Those samples are thrown
  away rather than averaged in.
* It runs each distance at two durations. If the car needs a moment to break
  away, distance is not proportional to time, and a rate fitted at one duration
  mispredicts every other one. Two points separate the startup dead time from
  the steady rate, and it reports both so you can see which one you have.

    ros2 run doggobot ...   no: this is interactive, run it directly
    python3 tools/calibrate_motion.py --forward
    python3 tools/calibrate_motion.py --turn
"""
import argparse
import json
import re
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


def parse_length(text):
    """Accept 58in, 4ft6, 1.47m, 147cm, 4'10\", or a bare number of inches."""
    t = text.strip().lower().replace(' ', '')
    if not t:
        return None
    m = re.fullmatch(r"(\d+(?:\.\d+)?)(?:ft|')(\d+(?:\.\d+)?)(?:in|\")?", t)
    if m:
        return (float(m.group(1)) * 12 + float(m.group(2))) * 0.0254
    m = re.fullmatch(r'(\d+(?:\.\d+)?)(m|cm|mm|in|"|ft|\')?', t)
    if not m:
        return None
    v = float(m.group(1))
    unit = m.group(2) or 'in'
    return {'m': v, 'cm': v / 100, 'mm': v / 1000, 'in': v * 0.0254,
            '"': v * 0.0254, 'ft': v * 0.3048, "'": v * 0.3048}[unit]


class Cal(Node):
    def __init__(self):
        super().__init__('calibrate_motion')
        self.cmd = self.create_publisher(String, 'voice_cmd', 10)
        self.arm = self.create_publisher(Bool, 'arm', 10)
        self.blocked = None
        self.sectors = {}
        self.vetoed = False
        self.create_subscription(String, 'obstacle_state', self._obs, 10)

    def _obs(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return
        self.sectors = d.get('sectors') or {}
        self.blocked = d.get('blocked')
        if self.blocked:
            self.vetoed = True

    def spin(self, seconds):
        end = time.time() + seconds
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def clearance(self):
        self.spin(0.6)
        return {k: v for k, v in self.sectors.items() if v is not None}

    def run(self, action, seconds):
        """Arm, run one primitive, disarm. Returns False if the guard cut in."""
        self.vetoed = False
        self.arm.publish(Bool(data=True))
        self.spin(0.4)
        self.cmd.publish(String(data=json.dumps(
            {'action': action, 'seconds': seconds, 'source': 'calibrate'})))
        self.spin(seconds + 1.2)
        self.arm.publish(Bool(data=False))
        self.spin(0.3)
        return not self.vetoed


def ask(prompt):
    try:
        return input(prompt)
    except EOFError:
        return ''


def forward(node):
    print(__doc__.split('    ros2 run')[0].strip())
    print('\n' + '=' * 68)
    print('FORWARD CALIBRATION')
    print('Measure from the same point every time. The rear wheel contact')
    print('patch is a good one: mark the floor at it before and after.')
    print('=' * 68)

    # The second duration is chosen AFTER the first pair, so the long run lands
    # around 1.4 m whatever the car actually does. A fixed 1.75 s would overrun a
    # 6 foot tape if the car turns out faster than the guess, and a measurement
    # you cannot take is worse than a shorter one.
    tape_m = parse_length(ask('length of your tape measure [6ft]: ') or '6ft')
    target = min(1.45, (tape_m or 1.83) * 0.80)
    print(f'  aiming the long run at about {target:.2f} m '
          f'({target / 0.0254:.0f} in), inside your tape.')

    runs = [(1.0, 2), (None, 2)]
    samples = []
    for secs, reps in runs:
        if secs is None:
            if samples:
                rate = sum(d / t for t, d in samples) / len(samples)
                secs = max(1.2, min(3.0, target / max(rate, 0.2)))
            else:
                secs = 1.75
            print(f'\n  measured about {sum(d / t for t, d in samples) / len(samples):.2f} m/s '
                  f'so far, so the long run will be {secs:.2f}s' if samples else '')
        for rep in range(1, reps + 1):
            while True:
                c = node.clearance()
                tight = {k: v for k, v in c.items() if v < 0.30}
                print(f'\n--- {secs:g}s run, rep {rep}/{reps} ---')
                print('  clearance: ' + ', '.join(f'{k} {v:.2f}m'
                                                  for k, v in sorted(c.items())))
                if tight:
                    print('  WARNING, the guard will stop the car mid-run: '
                          + ', '.join(f'{k} {v:.2f}m' for k, v in tight.items()))
                    print('  Move the car somewhere clearer, or override the')
                    print('  guard from the phone, then press enter.')
                if ask('  enter to run, s to skip: ').strip().lower() == 's':
                    break
                ok = node.run('forward', secs)
                if not ok:
                    print('  DISCARDED: the LiDAR guard cut in during the run,')
                    print('  so the car stopped early and the tape would lie.')
                    continue
                d = parse_length(ask('  measured distance (58in, 4ft6, 1.47m): '))
                if d is None:
                    print('  did not understand that, retrying')
                    continue
                print(f'  {d:.3f} m in {secs:g}s')
                samples.append((secs, d))
                break

    if len(samples) < 2:
        print('\nnot enough samples.')
        return 1

    # Simple rate, and a two-point fit that separates startup dead time.
    simple = sum(d / t for t, d in samples) / len(samples)
    by_t = {}
    for t, d in samples:
        by_t.setdefault(t, []).append(d)
    fit = None
    if len(by_t) >= 2:
        ts = sorted(by_t)
        t1, t2 = ts[0], ts[-1]
        d1 = sum(by_t[t1]) / len(by_t[t1])
        d2 = sum(by_t[t2]) / len(by_t[t2])
        if t2 > t1 and d2 > d1:
            v = (d2 - d1) / (t2 - t1)
            t0 = t1 - d1 / v
            fit = (v, t0)

    print('\n' + '=' * 68)
    print('RESULTS')
    for t, d in samples:
        print(f'  {t:g}s -> {d:.3f} m   ({d / t:.3f} m/s)')
    print(f'\n  simple average rate      : {simple:.3f} m/s')
    if fit:
        v, t0 = fit
        print(f'  steady rate (two-point)  : {v:.3f} m/s')
        print(f'  startup dead time        : {t0:+.3f} s')
        if abs(t0) < 0.15:
            print('\n  Dead time is small, so the simple rate is fine.')
            print(f'  Put this in config/behavior.yaml:\n')
            print(f'      metres_per_second: {simple:.2f}')
        else:
            print(f'\n  Dead time is NOT small. A rate alone will overshoot')
            print(f'  short distances and undershoot long ones. Use:\n')
            print(f'      metres_per_second: {v:.2f}')
            print(f'  and record the dead time as {t0:.2f}s, so the')
            print(f'  duration can be computed as t0 + metres/rate.')
    print('=' * 68)
    return 0


def turn(node):
    print('\n' + '=' * 68)
    print('TURN CALIBRATION')
    print('The car drives a circle. Watch the NOSE, and press enter the moment')
    print('it has come all the way back round to where it started.')
    print('Timing one full turn beats guessing an angle by eye.')
    print('=' * 68)

    samples = []
    for rep in (1, 2):
        c = node.clearance()
        print(f'\n--- revolution {rep}/2 ---')
        print('  clearance: ' + ', '.join(f'{k} {v:.2f}m'
                                          for k, v in sorted(c.items())))
        print('  It needs room on all sides for a full circle.')
        if ask('  enter to start, s to skip: ').strip().lower() == 's':
            continue
        node.arm.publish(Bool(data=True))
        node.spin(0.4)
        node.cmd.publish(String(data=json.dumps(
            {'action': 'circle_right', 'seconds': 30.0, 'source': 'calibrate'})))
        t0 = time.time()
        ask('  ... press enter at one full turn: ')
        elapsed = time.time() - t0
        node.cmd.publish(String(data=json.dumps(
            {'action': 'stop', 'source': 'calibrate'})))
        node.spin(0.4)
        node.arm.publish(Bool(data=False))
        node.spin(0.3)
        if node.vetoed:
            print('  the guard cut in during that circle, discarding')
            node.vetoed = False
            continue
        print(f'  {elapsed:.2f} s for 360 deg -> {360 / elapsed:.1f} deg/s')
        samples.append(360 / elapsed)

    if not samples:
        print('\nno samples.')
        return 1
    dps = sum(samples) / len(samples)
    print('\n' + '=' * 68)
    print(f'  degrees_per_second: {dps:.0f}')
    print(f'  turn_around_seconds: {180 / dps:.1f}   (180 deg at that rate)')
    print('=' * 68)
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--forward', action='store_true')
    ap.add_argument('--turn', action='store_true')
    a = ap.parse_args()
    if not (a.forward or a.turn):
        ap.error('pick --forward or --turn')

    rclpy.init()
    node = Cal()
    try:
        rc = 0
        if a.forward:
            rc |= forward(node)
        if a.turn:
            rc |= turn(node)
        return rc
    except KeyboardInterrupt:
        node.cmd.publish(String(data=json.dumps({'action': 'stop'})))
        node.arm.publish(Bool(data=False))
        node.spin(0.4)
        print('\nstopped and disarmed.')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
