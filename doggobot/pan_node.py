#!/usr/bin/env python3
"""Camera pan axis driver: /pan_cmd -> serial -> /pan_state.

Sole owner of the ESP32 serial port, for the same reason arbiter_node is the sole
/cmd_vel publisher. Three separate things want to aim this camera:

    follow tracking     keep the target centred in frame
    manual look         "atlas look left"
    search sweep        scan after losing a lock

Two of them writing to one serial port interleaves half-written command lines and
produces motion nobody asked for. So they do not get the port: behavior_node
arbitrates and emits a single /pan_cmd, exactly as it already arbitrates
/behavior_cmd, and this node just does as it is told.

SIGN CONVENTION, defined once, here, because three signs meet in the follow
cascade and any one of them backwards turns a converging loop into a diverging
one:

    x         > 0   target is RIGHT of frame centre   (perception_node)
    pan       > 0   camera points RIGHT of the chassis centreline  (this file)
    angular.z > 0   car steers RIGHT                  (verify per vehicle)

`invert` flips the servo direction without a reflash, because which way the
gearing faces is not known until the mount is bolted on.

Published state is a health signal as much as a measurement. `ok` goes false when
status lines stop arriving, so the follow cascade can fall back to fixed-camera
behaviour rather than steering the car on a pan angle frozen at whatever it last
happened to be.
"""
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    import serial
except ImportError:                                          # pragma: no cover
    serial = None


# The pan axis may never travel more than this either side of centre.
# Operator instruction 2026-09-01. Mirrored in firmware (ABS_LIMIT_DEG) so the
# bound survives a bad parameter, a bad message, and a crashed host alike.
PAN_CEILING_DEG = 90.0


class PanNode(Node):

    def __init__(self):
        super().__init__('pan_node')

        # By-id, not /dev/ttyUSB0: the LiDAR already claims ttyUSB0 on this Pi,
        # and USB enumeration order is not a promise. See deploy/99-doggobot-serial.rules
        self.declare_parameter('port', '/dev/doggobot-pan')
        self.declare_parameter('fallback_port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('invert', False)
        self.declare_parameter('limit_deg', 75.0)
        self.declare_parameter('centre_offset_deg', 0.0)
        self.declare_parameter('slew_deg_s', 200.0)
        # Leave false while bringing up a new mount: the axis then stays limp so
        # the horn can be turned by hand to find straight ahead.
        self.declare_parameter('engage_on_start', True)
        self.declare_parameter('publish_hz', 20.0)
        self.declare_parameter('stale_s', 0.5)
        self.declare_parameter('min_command_delta_deg', 0.4)
        self.declare_parameter('reconnect_s', 2.0)

        g = self.get_parameter
        self.port = g('port').value
        self.fallback_port = g('fallback_port').value
        self.baud = int(g('baud').value)
        self.invert = bool(g('invert').value)
        # +/-90 is a hardware ceiling, not a preference. A config asking for
        # more is a mistake, so it is clamped and said out loud rather than
        # quietly honoured. The firmware enforces the same bound independently.
        self.limit = min(float(g('limit_deg').value), PAN_CEILING_DEG)
        if float(g('limit_deg').value) > PAN_CEILING_DEG:
            self.get_logger().error(
                f"limit_deg {g('limit_deg').value} exceeds the "
                f"{PAN_CEILING_DEG:g} degree travel ceiling; using "
                f"{PAN_CEILING_DEG:g}")
        self.offset = float(g('centre_offset_deg').value)
        self.slew = float(g('slew_deg_s').value)
        self.engage = bool(g('engage_on_start').value)
        self.stale_s = float(g('stale_s').value)
        self.min_delta = float(g('min_command_delta_deg').value)
        self.reconnect_s = float(g('reconnect_s').value)

        self.ser = None
        self.lock = threading.Lock()
        self.deg = None
        self.target = 0.0
        self.moving = 0
        self.volts = 0.0
        self.temp = 0
        self.errs = 0
        self.last_line = 0.0
        self.last_sent = None
        self.running = True

        self.state_pub = self.create_publisher(String, 'pan_state', 10)
        self.create_subscription(Float32, 'pan_cmd', self._on_cmd, 10)

        self.reader = threading.Thread(target=self._reader, daemon=True)
        self.reader.start()
        self.create_timer(1.0 / float(g('publish_hz').value), self._publish)

        self.get_logger().info(
            f'pan: {self.port} @ {self.baud}, limit +/-{self.limit:g} deg, '
            f'invert={self.invert}')

    # -- serial ---------------------------------------------------------------

    def _open(self):
        if serial is None:
            self.get_logger().error('pyserial not installed: pip install pyserial')
            return None
        for path in (self.port, self.fallback_port):
            if not path:
                continue
            try:
                s = serial.Serial(path, self.baud, timeout=0.2)
                time.sleep(2.0)          # the ESP32 reboots when DTR asserts
                s.reset_input_buffer()
                self.get_logger().info(f'pan axis connected on {path}')
                # Hand the firmware its working configuration, in this order.
                # The offset goes FIRST so that the clamp the firmware applies
                # is measured from mechanical straight ahead rather than from
                # wherever the encoder happens to call centre, and the torque
                # goes last because the firmware boots limp on purpose and
                # engaging it holds the current angle rather than moving.
                s.write(f'o {self.offset}\n'.encode())
                s.write(f'v {self.slew}\n'.encode())
                if self.engage:
                    s.write(b'e 1\n')
                return s
            except Exception as e:                           # noqa: BLE001
                self.get_logger().warn(f'{path}: {e}')
        return None

    def _reader(self):
        buf = ''
        while self.running:
            if self.ser is None:
                self.ser = self._open()
                if self.ser is None:
                    time.sleep(self.reconnect_s)
                    continue
            try:
                chunk = self.ser.read(256).decode('utf-8', 'replace')
            except Exception as e:                           # noqa: BLE001
                self.get_logger().warn(f'pan serial lost: {e}')
                try:
                    self.ser.close()
                except Exception:                            # noqa: BLE001
                    pass
                self.ser = None
                continue
            if not chunk:
                continue
            buf += chunk
            while '\n' in buf:
                line, buf = buf.split('\n', 1)
                self._parse(line.strip())

    def _parse(self, line):
        if not line:
            return
        if line.startswith('#'):
            self.get_logger().info(f'pan fw: {line[1:].strip()}')
            return
        if not line.startswith('s '):
            return
        p = line.split()
        if len(p) < 8:
            return
        try:
            raw = None if p[1] == 'nan' else float(p[1])
            with self.lock:
                self.deg = None if raw is None else self._from_servo(raw)
                self.moving = int(p[3])
                self.volts = float(p[5])
                self.temp = int(p[6])
                self.errs = int(p[7])
                self.last_line = time.time()
        except ValueError:
            return

    # -- frame conversion -----------------------------------------------------
    # Chassis frame is what the rest of the stack speaks. Servo frame is what the
    # firmware speaks. The offset and inversion live on this boundary and nowhere
    # else, so there is exactly one place to look when the camera aims wrong.

    # The OFFSET now lives in the firmware, handed over once at connect. That is
    # deliberate: the firmware's travel clamp is the last line of defence for the
    # camera cable, and a clamp is only meaningful in the frame where straight
    # ahead is actually straight ahead. Leaving the offset up here would have
    # meant the firmware limiting +/-80 degrees about the wrong point.
    # Only the direction flip stays on this side.

    def _to_servo(self, chassis_deg):
        return -chassis_deg if self.invert else chassis_deg

    def _from_servo(self, servo_deg):
        return -servo_deg if self.invert else servo_deg

    # -- command --------------------------------------------------------------

    def _on_cmd(self, msg):
        want = float(msg.data)
        if not math.isfinite(want):
            return
        want = max(-self.limit, min(self.limit, want))
        self.target = want

        # Do not spam the link with sub-degree corrections it cannot resolve.
        # At 20 Hz an unfiltered PD output would send 20 near-identical lines a
        # second, each costing a bus round trip, for motion below the servo's own
        # resolution.
        if self.last_sent is not None and abs(want - self.last_sent) < self.min_delta:
            return

        if self.ser is None:
            return
        try:
            self.ser.write(f'p {self._to_servo(want):.2f}\n'.encode())
            self.last_sent = want
        except Exception as e:                               # noqa: BLE001
            self.get_logger().warn(f'pan write failed: {e}')
            self.ser = None

    # -- state ----------------------------------------------------------------

    def _publish(self):
        with self.lock:
            fresh = (time.time() - self.last_line) < self.stale_s
            payload = {
                'ok': bool(fresh and self.deg is not None),
                'deg': round(self.deg, 2) if self.deg is not None else None,
                'target': round(self.target, 2),
                'moving': self.moving,
                'volts': self.volts,
                'temp': self.temp,
                'errs': self.errs,
                'stamp': time.time(),
            }
        self.state_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        self.running = False
        if self.ser is not None:
            try:
                self.ser.write(b'p 0\n')     # leave the camera looking forward
                time.sleep(0.1)
                self.ser.close()
            except Exception:                                # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
