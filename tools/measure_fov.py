"""Measure half_fov_deg by moving the camera a known amount and watching x.

x = (bearing - pan) / half_fov, so with a stationary target dx/dpan = -1/half_fov.
The pan angle is encoder-verified to +/-90, which makes it a ruler we already
trust. No tape, no floor marks, no standing at a measured bearing.
"""
import json, sys, time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

ANGLES = [0.0, -12.0, -6.0, 0.0, 6.0, 12.0, 0.0]
SETTLE, SAMPLE = 1.6, 1.4

class F(Node):
    def __init__(self):
        super().__init__('fov_cal')
        self.pan = self.create_publisher(Float32, 'pan_manual', 10)
        self.lock = self.create_publisher(String, 'target_lock', 10)
        self.arm = self.create_publisher(Bool, 'arm', 10)
        self.x = None; self.status = None; self.locked = False
        self.pdeg = None; self.pok = False
        self.create_subscription(String, 'target_state', self.t, 10)
        self.create_subscription(String, 'pan_state', self.p, 10)
    def t(self, m):
        try: d = json.loads(m.data)
        except Exception: return
        self.locked = bool(d.get('locked')); self.status = d.get('status')
        self.x = d.get('x')
    def p(self, m):
        try: d = json.loads(m.data)
        except Exception: return
        self.pok = bool(d.get('ok')); self.pdeg = d.get('deg')
    def spin(self, s):
        e = time.time() + s
        while time.time() < e and rclpy.ok(): rclpy.spin_once(self, timeout_sec=0.02)

rclpy.init(); n = F(); n.spin(1.5)
for p in (n.pan, n.lock, n.arm):
    t0 = time.time()
    while p.get_subscription_count() == 0 and time.time() - t0 < 8: n.spin(0.1)

n.arm.publish(Bool(data=True)); n.spin(0.5)
n.lock.publish(String(data=json.dumps({'action': 'lock'}))); n.spin(2.5)
if not n.locked:
    print('no target locked. Stand in front of the camera and rerun.')
    n.destroy_node(); rclpy.shutdown(); sys.exit(1)
print(f'locked, status {n.status}\n')
print(f'{"pan cmd":>8} {"pan meas":>9} {"x mean":>8} {"n":>4}')
print('-' * 34)

pts = []
for a in ANGLES:
    n.pan.publish(Float32(data=a)); n.spin(SETTLE)
    xs, pd = [], []
    end = time.time() + SAMPLE
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.02)
        if n.status == 'TRACKED' and n.x is not None and n.pok and n.pdeg is not None:
            xs.append(n.x); pd.append(n.pdeg)
    if len(xs) < 5:
        print(f'{a:8.1f} {"-":>9} {"lost":>8} {len(xs):4d}')
        continue
    xm = sum(xs) / len(xs); pm = sum(pd) / len(pd)
    print(f'{a:8.1f} {pm:9.2f} {xm:8.3f} {len(xs):4d}')
    pts.append((pm, xm))

n.pan.publish(Float32(data=0.0)); n.spin(0.6)
n.lock.publish(String(data=json.dumps({'action': 'release'}))); n.spin(0.3)
n.arm.publish(Bool(data=False)); n.spin(0.4)

if len(pts) < 3:
    print('\nnot enough usable points.'); n.destroy_node(); rclpy.shutdown(); sys.exit(1)

mp = sum(p for p, _ in pts) / len(pts)
mx = sum(x for _, x in pts) / len(pts)
sxx = sum((p - mp) ** 2 for p, _ in pts)
sxy = sum((p - mp) * (x - mx) for p, x in pts)
slope = sxy / sxx                      # dx/dpan, expect negative
print(f'\nslope dx/dpan = {slope:+.5f} per degree')
if slope >= -1e-6:
    print('slope is not negative: the target moved, or the camera did not.')
else:
    hf = -1.0 / slope
    resid = max(abs(x - (mx + slope * (p - mp))) for p, x in pts)
    print(f'worst residual = {resid:.3f} in x')
    print(f'\n  half_fov_deg = {hf:.1f}   (currently configured 34.5)')
n.destroy_node(); rclpy.shutdown()
