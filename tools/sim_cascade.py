#!/usr/bin/env python3
"""Closed-loop simulation of the follow cascade, with no ROS and no car.

This drives the REAL FollowNode by stubbing rclpy, rather than reimplementing
the control law. A test that reimplements the thing it is testing proves only
that two copies of the same mistake agree.

What it is actually for: three signs meet in the cascade (bbox x, pan angle,
steering) and any one of them backwards turns convergence into divergence. On
hardware that costs a car into a wall. Here it costs nothing, and the last
scenario deliberately breaks a sign to show the test can see it.

    ./sim_cascade.py            run all scenarios
    ./sim_cascade.py --plot     ascii trace of bearing over time
"""
import argparse
import math
import sys
import types

# ---------------------------------------------------------------------------
# rclpy stubs. Just enough for FollowNode to construct and run.
# ---------------------------------------------------------------------------

OVERRIDES = {}


class _P:
    def __init__(self, v):
        self.value = v


class _Log:
    quiet = True

    def _emit(self, tag, m):
        if not _Log.quiet:
            print(f'  [{tag}] {m}')

    def info(self, m):
        self._emit('info', m)

    def warn(self, m):
        self._emit('warn', m)

    def error(self, m):
        self._emit('ERR ', m)


class _Pub:
    def __init__(self, node, topic):
        self.node, self.topic = node, topic

    def publish(self, msg):
        self.node.sent.setdefault(self.topic, []).append(msg)


class _Node:
    def __init__(self, name):
        self._p = {}
        self.sent = {}

    def declare_parameter(self, n, v):
        self._p[n] = _P(OVERRIDES.get(n, v))

    def get_parameter(self, n):
        return self._p[n]

    def create_publisher(self, t, topic, q):
        return _Pub(self, topic)

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return _Log()

    def add_on_set_parameters_callback(self, cb):
        self._param_cb = cb

    def destroy_node(self):
        pass


class _Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class _Data:
    def __init__(self, data=None):
        self.data = data


def _install_stubs():
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: True
    node_mod = types.ModuleType('rclpy.node')
    node_mod.Node = _Node
    rclpy.node = node_mod

    geo = types.ModuleType('geometry_msgs')
    geo_msg = types.ModuleType('geometry_msgs.msg')
    geo_msg.Twist = _Twist
    geo.msg = geo_msg

    std = types.ModuleType('std_msgs')
    std_msg = types.ModuleType('std_msgs.msg')
    std_msg.String = _Data
    std_msg.Float32 = _Data
    std_msg.Bool = _Data
    std.msg = std_msg

    for name, mod in (('rclpy', rclpy), ('rclpy.node', node_mod),
                      ('geometry_msgs', geo), ('geometry_msgs.msg', geo_msg),
                      ('std_msgs', std), ('std_msgs.msg', std_msg)):
        sys.modules[name] = mod


_install_stubs()

import json                                                  # noqa: E402
import os                                                    # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from doggobot.follow_node import FollowNode                   # noqa: E402

# ---------------------------------------------------------------------------
# Plant
# ---------------------------------------------------------------------------

DT = 0.01                    # physics step
FRAME_DT = 1.0 / 12.3        # detector rate, measured on the OAK-D Lite
CAPTURE_LAG = 0.08           # frame is this old by the time follow sees it
PAN_DT = 1.0 / 20.0          # pan_state publish rate

WHEELBASE = 0.30             # m
MAX_DELTA = math.radians(30)  # steering angle at angular.z = 1.0
# The steering servo does not move instantly, and the first version of this
# model pretended it did. That mattered: with instant steering NO gain in the
# sweep produced any overshoot, while the real car visibly overshoots, so the
# model was missing the very thing being tuned. A rate limit plus a first-order
# lag is the cheapest honest approximation of an RC steering servo.
STEER_TAU = 0.10             # s
STEER_RATE = 4.0             # full -1..1 travel in about 0.5 s
# The car has mass, and the model did not. DERIVED FROM MEASUREMENT rather than
# invented: the car coasts 0.144 m after a command ends at 0.635 m/s, so
# a = v^2/2d = 1.40 m/s^2. Without this the vehicle stops dead the instant the
# controller says to, which is precisely why no gain in the sweep could ever
# overshoot a standoff.
ACCEL = 1.40                 # m/s^2, from the measured coast distance
# MEASURED on the car, not guessed. 0.365 throttle produced 0.635 m/s
# (2026-09-02 tape measure), so the scale is 0.635/0.365.
SPEED_PER_THROTTLE = 1.74
PAN_SLEW = 120.0             # deg/s, MEASURED on the bench 2026-08-30
# Measured settling: 30 deg in 0.50 s, 120 deg in 1.00 s. A first-order
# constant of about 0.25 s reproduces that alongside the rate limit.
PAN_TAU = 0.25               # s
HALF_FOV = 25.2              # deg, MEASURED 2026-09-02, see config/follow.yaml


class World:
    """Bicycle-model car, rate-limited camera, a target that may walk."""

    def __init__(self, target=(3.0, 2.5), target_vel=(0.0, 0.0), use_pan=True):
        self.x = self.y = self.th = 0.0
        self.tx, self.ty = target
        self.tvx, self.tvy = target_vel
        self.pan = 0.0            # actual camera angle, chassis frame, deg
        self.pan_cmd = 0.0
        self.steer_actual = 0.0   # what the servo managed, not what was asked
        self.v = 0.0              # actual speed, which lags the demand
        self.use_pan = use_pan
        self.t = 0.0
        self.throttle = self.steer = 0.0
        self.history = []         # (t, bearing, range, pan, visible)

    # -- geometry --
    def bearing(self):
        """Angle to target in the CHASSIS frame, degrees, +ve = right."""
        dx, dy = self.tx - self.x, self.ty - self.y
        world = math.atan2(dy, dx)
        rel = math.atan2(math.sin(world - self.th), math.cos(world - self.th))
        # +ve chassis-right is a NEGATIVE mathematical angle (y is left)
        return -math.degrees(rel)

    def range_m(self):
        return math.hypot(self.tx - self.x, self.ty - self.y)

    def visible(self):
        """In frame if the target sits within the camera's half-FOV of pan."""
        return abs(self.bearing() - self.pan) <= HALF_FOV

    def step(self):
        self.tx += self.tvx * DT
        self.ty += self.tvy * DT

        # Servo lag: approach the demand with a time constant, rate limited.
        err = self.steer - self.steer_actual
        move = err * (DT / STEER_TAU)
        lim = STEER_RATE * DT
        self.steer_actual += max(-lim, min(lim, move))
        v_want = self.throttle * SPEED_PER_THROTTLE
        dv = v_want - self.v
        lim = ACCEL * DT
        self.v += max(-lim, min(lim, dv))
        v = self.v
        delta = -self.steer_actual * MAX_DELTA  # +ve steer = right = -ve yaw
        self.th += (v * math.tan(delta) / WHEELBASE) * DT
        self.x += v * math.cos(self.th) * DT
        self.y += v * math.sin(self.th) * DT

        if self.use_pan:
            # Rate limit AND a settling lag. A pure rate limit reaches the
            # target exactly on time, which hides the pile-up that an
            # incremental command suffers while the servo is still moving.
            err = self.pan_cmd - self.pan
            move = err * (DT / PAN_TAU)
            lim = PAN_SLEW * DT
            self.pan += max(-lim, min(lim, move))
        self.t += DT


# ---------------------------------------------------------------------------
# Scenario runner
# ---------------------------------------------------------------------------

def run(name, seconds=14.0, use_pan=True, target=(3.0, 2.5),
        target_vel=(0.0, 0.0), overrides=None, verbose=False):
    global OVERRIDES
    OVERRIDES = {'debug_hz': 0.0, 'use_pan': use_pan,
                 'half_fov_deg': HALF_FOV, 'kp_steer': 0.6, 'kd_steer': 0.1,
                 'steer_deadband': 0.05, 'max_steer': 0.7,
                 'standoff_mm': 1000.0, 'kp_throttle': 0.0004,
                 'kd_throttle': 0.0001, 'throttle_deadband_mm': 300.0,
                 'throttle_floor': 0.365, 'max_throttle': 0.38,
                 'allow_reverse': False}
    OVERRIDES.update(overrides or {})
    _Log.quiet = not verbose

    node = FollowNode()
    w = World(target=target, target_vel=target_vel, use_pan=use_pan)

    frames = [] if not use_pan else None
    next_frame = 0.0
    next_pan = 0.0
    lost_frames = 0
    total_frames = 0
    # a small queue so the detector output arrives CAPTURE_LAG late
    pending = []

    while w.t < seconds:
        # camera state to the follow node at 20 Hz
        if w.t >= next_pan:
            next_pan += PAN_DT
            node._on_pan(_Data(json.dumps(
                {'ok': True, 'deg': round(w.pan, 2), 'target': w.pan_cmd,
                 'moving': 0, 'volts': 7.9, 'temp': 30, 'errs': 0,
                 'stamp': w.t})))

        # detector: capture now, deliver CAPTURE_LAG later
        if w.t >= next_frame:
            next_frame += FRAME_DT
            total_frames += 1
            vis = w.visible()
            if not vis:
                lost_frames += 1
            x = (w.bearing() - w.pan) / HALF_FOV if vis else None
            pending.append((w.t + CAPTURE_LAG,
                            {'locked': True,
                             'status': 'TRACKED' if vis else 'LOST',
                             'x': max(-1.0, min(1.0, x)) if vis else 0.0,
                             'z_mm': int(w.range_m() * 1000),
                             'id': 1, 'stamp': w.t}))

        while pending and pending[0][0] <= w.t:
            _, payload = pending.pop(0)
            node.sent.clear()
            node._on_target(_Data(json.dumps(payload)))
            for m in node.sent.get('follow_cmd', []):
                w.throttle, w.steer = m.linear.x, m.angular.z
            for m in node.sent.get('follow_pan', []):
                w.pan_cmd = m.data

        w.history.append((w.t, w.bearing(), w.range_m(), w.pan, w.visible()))
        w.step()

    return node, w, lost_frames, total_frames


def camera_hunt(w, after=1.0):
    """How much the CAMERA oscillates about the target: sign changes of the
    aiming error, and the worst error once tracking should have settled."""
    h = [(b - p) for t, b, _, p, _ in w.history if t > after]
    if len(h) < 3:
        return 0, 0.0
    flips = sum(1 for i in range(1, len(h)) if h[i - 1] * h[i] < 0)
    return flips, max(abs(e) for e in h)


def swings(w, after=1.0):
    """Overshoot metrics: sign changes of the bearing, and the worst excursion
    after the first crossing. A loop that settles crosses zero once; one that
    overshoots crosses repeatedly and keeps a large excursion each time."""
    h = [(t, b) for t, b, _, _, _ in w.history if t > after]
    if len(h) < 3:
        return 0, 0.0
    crossings, peak, seen = 0, 0.0, False
    for i in range(1, len(h)):
        if h[i - 1][1] * h[i][1] < 0:
            crossings += 1
            seen = True
        if seen:
            peak = max(peak, abs(h[i][1]))
    return crossings, peak


def report(name, w, lost, total, expect):
    bearing = w.bearing()
    rng = w.range_m()
    peak_pan = max(abs(p) for _, _, _, p, _ in w.history)
    lost_pct = 100.0 * lost / max(1, total)
    aim_err = bearing - w.pan          # how far off the CAMERA is, not the chassis

    # What the follow controller is actually responsible for: sit at the standoff
    # and keep the target framed. NOT "point the chassis at the target" - once the
    # car stops at the standoff it cannot change heading at all, because a
    # non-holonomic vehicle only turns while it is moving. Gating on chassis
    # bearing would mark correct behaviour as failure.
    ranged = abs(rng - 1.0) < 0.45
    framed = abs(aim_err) < 15.0 and lost_pct < 10.0
    good = ranged and framed
    ok = good if expect == 'converge' else not good

    print(f'  {name:<32} rng {rng:4.2f} m  chassis {bearing:+6.1f}  '
          f'camera off {aim_err:+6.1f}  pan used {peak_pan:5.1f}  '
          f'lost {lost_pct:5.1f}%   {"PASS" if ok else "FAIL"}')
    return ok


def plot(w, rows=16, cols=70):
    hist = w.history[::max(1, len(w.history) // cols)][:cols]
    lo, hi = -70.0, 70.0
    print('\n  bearing over time (+ right, - left), 0 = aimed at target')
    for r in range(rows):
        hi_r = hi - (hi - lo) * r / rows
        lo_r = hi - (hi - lo) * (r + 1) / rows
        line = ''.join('#' if lo_r <= b < hi_r else
                       ('-' if lo_r <= 0 < hi_r else ' ')
                       for _, b, _, _, _ in hist)
        print(f'  {hi_r:+5.0f} |{line}')


def anchor_ab():
    """Incremental vs anchored inner loop, on cases that stress the camera."""
    print('camera inner loop: increment onto last command vs anchor to '
          'measured angle\n')
    cases = [
        ('person crosses close and fast', (2.0, 0.0), (0.0, 1.0), 10.0),
        ('person orbits a stopped car',   (1.0, 0.0), (0.0, 0.5), 10.0),
        ('approach from 20 deg',          (2.8, 1.02), (0.0, 0.0), 12.0),
    ]
    print(f'  {"scenario":<32} {"mode":>10} {"flips":>6} {"worst aim":>10}'
          f' {"lost":>6}')
    print('  ' + '-' * 70)
    for name, tgt, vel, secs in cases:
        for label, anchor in (('increment', False), ('anchored', True)):
            _, w, l, t = run('ab', use_pan=True, target=tgt, target_vel=vel,
                             seconds=secs,
                             overrides={'pan_anchor_to_measured': anchor})
            f, worst = camera_hunt(w)
            print(f'  {name if label == "increment" else "":<32} {label:>10}'
                  f' {f:6d} {worst:9.1f}d {100.0 * l / max(1, t):5.1f}%')
        print()
    return 0


SWEEP_KEY = 'kp_pan'
SWEEP_VALS = (0.40, 0.55, 0.70, 0.85, 1.00)


def gain_sweep():
    """Sweep SWEEP_KEY over SWEEP_VALS, reporting camera hunt and lag."""
    print(f'anchored inner loop: {SWEEP_KEY} vs lag and oscillation\n')
    cases = [('crosses fast', (2.0, 0.0), (0.0, 1.0), 10.0),
             ('orbits stopped car', (1.0, 0.0), (0.0, 0.5), 10.0),
             ('static, 20 deg off', (2.8, 1.02), (0.0, 0.0), 12.0)]
    print(f'  {SWEEP_KEY[:7]:>7} ' + ' '.join(f'{n[:18]:>22}' for n, _, _, _ in cases))
    print(f'  {"":>7} ' + ' '.join(f'{"flips / worst aim":>22}' for _ in cases))
    print('  ' + '-' * 78)
    for ki in SWEEP_VALS:
        cells = []
        for _, tgt, vel, secs in cases:
            _, w, l, t = run('ki', use_pan=True, target=tgt, target_vel=vel,
                             seconds=secs,
                             overrides=dict({'pan_anchor_to_measured': True},
                                            **{SWEEP_KEY: ki}))
            f, worst = camera_hunt(w)
            cells.append(f'{f:6d} / {worst:6.1f}d')
        print(f'  {ki:7.2f} ' + ' '.join(f'{c:>22}' for c in cells))
    return 0


def sweep():
    """Grid over the loop gains, ranked by how much the car oscillates."""
    target = (2.8, 1.02)
    print('gain sweep: approach from 20 deg, measuring overshoot\n')
    print(f'  {"kp_steer":>8} {"kd_steer":>8} {"kp_pan":>7} {"kd_pan":>7}'
          f' {"crossings":>10} {"peak|brg|":>10} {"rng":>6} {"camera":>7}')
    print('  ' + '-' * 70)
    rows = []
    for kps in (0.60, 0.45, 0.35, 0.25):
        for kds in (0.10, 0.20, 0.30):
            for kpp, kdp in ((0.55, 0.05), (0.40, 0.10)):
                _, w, l, t = run('g', use_pan=True, target=target,
                                 overrides={'kp_steer': kps, 'kd_steer': kds,
                                            'kp_pan': kpp, 'kd_pan': kdp})
                c, pk = swings(w)
                aim = abs(w.bearing() - w.pan)
                rows.append((c, pk, kps, kds, kpp, kdp, w.range_m(), aim))
                print(f'  {kps:8.2f} {kds:8.2f} {kpp:7.2f} {kdp:7.2f}'
                      f' {c:10d} {pk:10.1f} {w.range_m():6.2f} {aim:7.1f}')
    rows.sort(key=lambda r: (r[0], r[1]))
    c, pk, kps, kds, kpp, kdp, rng, aim = rows[0]
    print(f'\n  calmest: kp_steer {kps}, kd_steer {kds}, kp_pan {kpp}, '
          f'kd_pan {kdp}')
    print(f'           {c} crossings, peak {pk:.1f} deg, range {rng:.2f} m, '
          f'camera {aim:.1f} deg off')
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plot', action='store_true')
    ap.add_argument('--sweep', action='store_true')
    ap.add_argument('--anchor-ab', action='store_true')
    ap.add_argument('--ki-sweep', action='store_true')
    ap.add_argument('-v', '--verbose', action='store_true')
    a = ap.parse_args()
    if a.ki_sweep:
        return gain_sweep()
    if a.anchor_ab:
        return anchor_ab()
    if a.sweep:
        return sweep()

    print('follow cascade simulation')
    print(f'  plant: bicycle model, {WHEELBASE} m wheelbase, '
          f'pan slew {PAN_SLEW:.0f} deg/s, detector {1/FRAME_DT:.1f} fps, '
          f'{CAPTURE_LAG*1000:.0f} ms capture lag\n')

    results = []

    # Target must START inside the half-FOV: acquiring something the camera
    # cannot see is the search sweep's job, not the follow loop's.
    # Both must START inside the half-FOV, which is 25.2 deg now that it has
    # been measured rather than taken from the spec sheet. The older geometry
    # (28 and 31 deg) sat outside it, so the car simply never saw the target and
    # every configuration produced identical do-nothing results.
    OFFSET = (2.8, 1.02)     # ~20 deg off the nose, 3.0 m out
    TIGHT = (1.5, 0.55)      # ~20 deg off, 1.6 m out: forces a hard turn

    print('  wide approach (target 20 deg off, 3.0 m):')
    n, w, l, t = run('a', use_pan=True, target=OFFSET, verbose=a.verbose)
    results.append(report('cascade', w, l, t, 'converge'))
    first = w

    n, w, l, t = run('b', use_pan=False, target=OFFSET)
    results.append(report('fixed camera', w, l, t, 'converge'))
    fixed = w

    print('\n  tight turn (target 20 deg off, 1.6 m, small room):')
    n, w, l, t = run('c', use_pan=True, target=TIGHT)
    results.append(report('cascade', w, l, t, 'converge'))
    tight_pan = w

    n, w, l, t = run('d', use_pan=False, target=TIGHT)
    tight_fixed = w
    report('fixed camera', w, l, t, 'converge')      # informational, not a gate

    print('\n  target walking across at 0.6 m/s (car tops out near 0.9):')
    n, w, l, t = run('e', use_pan=True, target=(3.0, 0.0), target_vel=(0.0, 0.6))
    results.append(report('cascade', w, l, t, 'converge'))
    n, w, l, t = run('f', use_pan=False, target=(3.0, 0.0), target_vel=(0.0, 0.6))
    report('fixed camera', w, l, t, 'converge')      # informational, not a gate

    # The case the pan axis exists for. The car is already AT the standoff, so it
    # is stopped, so its heading is frozen. Everything the target does from here
    # is the camera's problem alone.
    print('\n  person walks around a car already stopped at the standoff:')
    # 0.5 m/s, not 0.8: the car's measured top speed is about 0.66 m/s, so a
    # faster walker simply outruns it and the scenario would be testing the
    # throttle rather than the camera.
    n, w, l, t = run('i', use_pan=True, target=(1.0, 0.0), target_vel=(0.0, 0.5),
                     seconds=8.0)
    results.append(report('cascade', w, l, t, 'converge'))
    orbit_cascade = 100.0 * l / max(1, t)
    n, w, l, t = run('j', use_pan=False, target=(1.0, 0.0), target_vel=(0.0, 0.5),
                     seconds=8.0)
    report('fixed camera', w, l, t, 'converge')      # informational, not a gate
    orbit_fixed = 100.0 * l / max(1, t)

    print('\n  sign-error controls (these SHOULD fail to converge):')
    n, w, l, t = run('g', use_pan=True, target=OFFSET,
                     overrides={'kp_pan': -0.55})
    results.append(report('pan sign inverted', w, l, t, 'diverge'))

    n, w, l, t = run('h', use_pan=True, target=OFFSET,
                     overrides={'kp_steer': -0.6})
    results.append(report('steering sign inverted', w, l, t, 'diverge'))

    print(f'\n  the point of it: with the car stopped at the standoff and the '
          f'person walking\n  around it, the fixed camera loses them for '
          f'{orbit_fixed:.0f}% of frames. The pan axis: {orbit_cascade:.0f}%.')

    if a.plot:
        print('\n--- cascade ---')
        plot(first)
        print('\n--- fixed camera ---')
        plot(fixed)

    print(f'\n{sum(results)}/{len(results)} checks passed')
    return 0 if all(results) else 1


if __name__ == '__main__':
    sys.exit(main())
