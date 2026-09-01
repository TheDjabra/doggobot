"""Minimal rclpy stand-ins so node logic can be tested without a ROS install.

Lets the real node classes be constructed, driven with synthetic messages and
observed, on any machine. The point is to test the SHIPPING code: a test that
reimplements the logic it checks proves only that two copies of one mistake
agree.
"""
import sys
import types

OVERRIDES = {}


class _P:
    def __init__(self, v):
        self.value = v


class _Log:
    quiet = True
    def __init__(self, sink=None):
        self.sink = sink
    def _emit(self, tag, m):
        if self.sink is not None:
            self.sink.append((tag, m))
        if not _Log.quiet:
            print(f'  [{tag}] {m}')
    def info(self, m):  self._emit('info', m)
    def warn(self, m):  self._emit('warn', m)
    def error(self, m): self._emit('error', m)


class _Pub:
    def __init__(self, node, topic):
        self.node, self.topic = node, topic
    def publish(self, msg):
        self.node.sent.setdefault(self.topic, []).append(msg)


class Node:
    def __init__(self, name):
        self._p = {}
        self.sent = {}
        self.logs = []
        self._logger = _Log(self.logs)
    def declare_parameter(self, n, v): self._p[n] = _P(OVERRIDES.get(n, v))
    def get_parameter(self, n): return self._p[n]
    def create_publisher(self, t, topic, q): return _Pub(self, topic)
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return self._logger
    def destroy_node(self): pass


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


class Data:
    def __init__(self, data=None): self.data = data


class LaserScan:
    def __init__(self):
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0
        self.range_min = 0.01
        self.range_max = 12.0


def install():
    """Register the stub modules in sys.modules. Call before importing a node."""
    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda *a, **k: None
    rclpy.shutdown = lambda *a, **k: None
    rclpy.spin = lambda *a, **k: None
    rclpy.ok = lambda: True
    nm = types.ModuleType('rclpy.node'); nm.Node = Node
    rclpy.node = nm

    geo = types.ModuleType('geometry_msgs')
    gm = types.ModuleType('geometry_msgs.msg'); gm.Twist = Twist
    geo.msg = gm

    std = types.ModuleType('std_msgs')
    sm = types.ModuleType('std_msgs.msg')
    sm.String = Data; sm.Float32 = Data; sm.Bool = Data
    std.msg = sm

    sen = types.ModuleType('sensor_msgs')
    senm = types.ModuleType('sensor_msgs.msg'); senm.LaserScan = LaserScan
    sen.msg = senm

    for n, m in (('rclpy', rclpy), ('rclpy.node', nm),
                 ('geometry_msgs', geo), ('geometry_msgs.msg', gm),
                 ('std_msgs', std), ('std_msgs.msg', sm),
                 ('sensor_msgs', sen), ('sensor_msgs.msg', senm)):
        sys.modules[n] = m
