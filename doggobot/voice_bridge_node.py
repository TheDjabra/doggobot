#!/usr/bin/env python3
"""Phone control surface: FastAPI + WebSocket + rclpy in one process.

Serves the control page and turns what the phone sends into ROS2 messages:

    button / joystick  ->  /teleop_cmd    (geometry_msgs/Twist)
    kill switch        ->  /estop         (std_msgs/Bool)
    speech             ->  /voice_cmd     (std_msgs/String, JSON)  [not wired yet]

Why one process rather than rosbridge: a server has to exist anyway to serve the
page over HTTPS and (later) to proxy the LLM call so the key never reaches the
browser. Given that, rosbridge would add a second server and port for nothing.
rclpy spins in a background thread and the WebSocket handler calls into the node.

Safety, in order of how much it matters:

* The kill switch is a WebSocket frame straight to /estop. It never depends on
  speech recognition, a model, or a network hop to anything but this process.
* Teleop is a STREAM, not a command. The browser repeats the current input at
  send_hz and the arbiter drops teleop after its staleness timeout, so a phone
  that loses wifi, locks its screen, or is dropped releases the car within half
  a second without anyone sending a stop.
* When the last client disconnects, teleop stops being published. Same effect,
  borrowed from the turret app's rule of disarming the payload when the last
  viewer leaves.
"""
import asyncio
import json
import os
import threading

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, HTMLResponse, StreamingResponse
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32, String

def _web_dir():
    """Find index.html whether installed or running from a symlinked source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('doggobot'), 'web')
        if os.path.exists(os.path.join(share, 'index.html')):
            return share
    except Exception:                                # noqa: BLE001
        pass
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'web')


WEB_DIR = _web_dir()


class BridgeNode(Node):
    """The ROS2 half. Deliberately dumb: it publishes what it is told."""

    def __init__(self):
        super().__init__('voice_bridge_node')
        self.declare_parameter('port', 8080)
        self.declare_parameter('max_teleop_throttle', 0.30)
        self.declare_parameter('throttle_floor', 0.365)
        # Seconds to wait after the last client leaves before disarming. Phone
        # browsers close WebSockets constantly (screen sleep, backgrounding, a
        # network blip), and disarming instantly killed a running sequence three
        # seconds in. Walking away should still make the car inert, so this is a
        # delay rather than a removal.
        self.declare_parameter('disarm_grace_s', 10.0)

        self.port = int(self.get_parameter('port').value)
        self.max_teleop_throttle = float(
            self.get_parameter('max_teleop_throttle').value)
        self.throttle_floor = float(self.get_parameter('throttle_floor').value)

        self.teleop_pub = self.create_publisher(Twist, 'teleop_cmd', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)
        self.voice_pub = self.create_publisher(String, 'voice_cmd', 10)
        self.lock_pub = self.create_publisher(String, 'target_lock', 10)
        self.arm_pub = self.create_publisher(Bool, 'arm', 10)
        self.color_pub = self.create_publisher(String, 'color_config', 10)
        self.video_pub = self.create_publisher(String, 'video_config', 10)
        self.autonomy_pub = self.create_publisher(Bool, 'autonomy_enabled', 10)
        # NOT /pan_cmd. behavior_node is the sole writer to that topic and
        # arbitrates the camera between follow tracking and a manual look; the
        # phone is just another claimant and gets the same treatment as a spoken
        # "look left" rather than a private line to the servo.
        self.pan_pub = self.create_publisher(Float32, 'pan_manual', 10)

        # Latest perception state, pushed to the phone so the operator can see
        # WHAT the car is following rather than inferring it from behaviour.
        # Written from the rclpy thread, read from the asyncio loop: a plain
        # attribute assignment is atomic under the GIL, which is all we need.
        self.target = {'locked': False, 'status': 'UNKNOWN'}
        self.create_subscription(String, 'target_state', self._on_target, 10)

        # Which primitive is running, so the phone shows what the car is doing
        # rather than what the operator last asked for.
        self.behavior = {'active': None}
        self.create_subscription(String, 'behavior_state', self._on_behavior, 10)
        self.create_subscription(String, 'pan_state', self._on_pan, 10)

        self.arbiter = {'armed': False}
        self.create_subscription(String, 'arbiter/status', self._on_arbiter, 10)

        self.condition = {'color': None, 'areas': {}}
        self.pan = {'ok': False, 'deg': None, 'target': 0.0}
        self.create_subscription(
            String, 'condition_state', self._on_condition, 10)

        # Latest camera JPEG. Subscribing here is what makes perception_node
        # encode at all, so the stream costs nothing until this node exists.
        self.frame = None
        self.create_subscription(
            CompressedImage, 'camera/compressed', self._on_frame, 2)

        # NOT `self.clients`: rclpy.node.Node already defines `clients` as a
        # read-only property listing this node's service clients.
        self.client_count = 0
        self.disarm_grace = float(self.get_parameter('disarm_grace_s').value)
        self._disarm_timer = None
        self.get_logger().info(
            f'bridge up on :{self.port}, teleop ceiling '
            f'{self.max_teleop_throttle}, floor {self.throttle_floor}')

    def publish_teleop(self, throttle, steering):
        t = Twist()
        # Step over the motor's measured deadband: below the floor the car does
        # not creep, it stops, so a half-pressed button would feel dead.
        if 0.0 < abs(throttle) < self.throttle_floor:
            throttle = self.throttle_floor if throttle > 0 else -self.throttle_floor
        t.linear.x = max(-self.max_teleop_throttle,
                         min(self.max_teleop_throttle, float(throttle)))
        t.angular.z = max(-0.8, min(0.8, float(steering)))
        self.teleop_pub.publish(t)

    def publish_estop(self, engaged):
        self.estop_pub.publish(Bool(data=bool(engaged)))
        self.get_logger().warn(
            f'E-STOP {"ENGAGED" if engaged else "cleared"} from phone')

    def _on_target(self, msg):
        try:
            self.target = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def _on_behavior(self, msg):
        try:
            self.behavior = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def _on_frame(self, msg):
        self.frame = bytes(msg.data)

    def _on_pan(self, msg):
        try:
            self.pan = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def _on_condition(self, msg):
        try:
            self.condition = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def publish_video_config(self, cfg):
        self.video_pub.publish(String(data=json.dumps(cfg)))

    def publish_autonomy(self, enabled):
        self.autonomy_pub.publish(Bool(data=bool(enabled)))
        self.get_logger().info(
            f'autonomy {"enabled" if enabled else "suppressed"} from phone')

    def publish_color_config(self, cfg):
        self.color_pub.publish(String(data=json.dumps(cfg)))

    def _on_arbiter(self, msg):
        try:
            self.arbiter = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            pass

    def publish_arm(self, armed):
        self.arm_pub.publish(Bool(data=bool(armed)))
        self.get_logger().warn(f'{"ARMED" if armed else "disarmed"} from phone')

    def schedule_disarm(self):
        """Disarm after the grace period, unless somebody reconnects first."""
        self.cancel_disarm()
        self.get_logger().info(
            f'no clients; disarming in {self.disarm_grace:.0f}s unless one returns')
        self._disarm_timer = self.create_timer(
            self.disarm_grace, self._disarm_now)

    def _disarm_now(self):
        self.cancel_disarm()
        if self.client_count <= 0:
            self.publish_arm(False)

    def cancel_disarm(self):
        if self._disarm_timer is not None:
            self._disarm_timer.cancel()
            self.destroy_timer(self._disarm_timer)
            self._disarm_timer = None

    def publish_pan(self, deg):
        try:
            d = float(deg)
        except (TypeError, ValueError):
            return
        self.pan_pub.publish(Float32(data=max(-90.0, min(90.0, d))))

    def publish_lock(self, engage):
        action = 'lock' if engage else 'release'
        self.lock_pub.publish(String(data=json.dumps({'action': action})))
        self.get_logger().info(f'target {action} from phone')

    def publish_voice(self, text, source, alternatives=None):
        payload = {'text': text, 'source': source}
        if alternatives:
            # Browser recognition returns ranked guesses. Passing them along lets
            # behavior_node fall through to the next one when the top guess is
            # not a command, which recovers a lot of near-misses in a noisy room.
            payload['alternatives'] = alternatives
        self.voice_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'voice[{source}]: {text}')

    def publish_action(self, action, seconds=None):
        """A button press. Same topic as speech, so behavior_node stays the one
        place that knows what the car can do."""
        payload = {'action': action, 'source': 'phone-button'}
        if seconds:
            payload['seconds'] = seconds
        self.voice_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'button: {action}')


def build_app(node: BridgeNode) -> FastAPI:
    app = FastAPI(title='DOGGOBOT')

    @app.get('/', response_class=HTMLResponse)
    async def index():
        with open(os.path.join(WEB_DIR, 'index.html'), encoding='utf-8') as f:
            return f.read()

    # Files the browser needs to treat this as an installable app rather than a
    # web page. Served explicitly rather than by mounting a static directory, so
    # nothing else in the package is exposed.
    @app.get('/manifest.webmanifest')
    async def manifest():
        return FileResponse(os.path.join(WEB_DIR, 'manifest.webmanifest'),
                            media_type='application/manifest+json')

    @app.get('/favicon.svg')
    async def favicon():
        return FileResponse(os.path.join(WEB_DIR, 'favicon.svg'),
                            media_type='image/svg+xml')

    @app.get('/icon-192.png')
    async def icon192():
        return FileResponse(os.path.join(WEB_DIR, 'icon-192.png'))

    @app.get('/icon-512.png')
    async def icon512():
        return FileResponse(os.path.join(WEB_DIR, 'icon-512.png'))

    @app.get('/apple-touch-icon.png')
    async def appleicon():
        return FileResponse(os.path.join(WEB_DIR, 'apple-touch-icon.png'))

    @app.get('/stream.mjpg')
    async def stream():
        """MJPEG: the oldest trick, and the one that needs no client library.

        An <img> pointed at this renders live video in any browser with no
        JavaScript, no WebRTC negotiation, and no codec support to check.
        """
        async def frames():
            last = None
            while True:
                buf = node.frame
                if buf is not None and buf is not last:
                    last = buf
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(buf)).encode()
                           + b'\r\n\r\n' + buf + b'\r\n')
                await asyncio.sleep(0.05)

        return StreamingResponse(
            frames(),
            media_type='multipart/x-mixed-replace; boundary=frame')

    @app.get('/healthz')
    async def healthz():
        return {'ok': True, 'clients': node.client_count}

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        node.client_count += 1
        node.cancel_disarm()
        node.get_logger().info(f'client connected ({node.client_count} total)')

        async def push_target():
            """Stream perception state to the phone at 5 Hz.

            Deliberately slower than perception's ~12 Hz: the operator is reading
            it, not controlling on it, and this runs over a phone link.
            """
            try:
                while True:
                    await sock.send_text(json.dumps(
                        {'type': 'target', **node.target}))
                    await sock.send_text(json.dumps(
                        {'type': 'behavior', **node.behavior}))
                    await sock.send_text(json.dumps(
                        {'type': 'arbiter', **node.arbiter}))
                    await sock.send_text(json.dumps(
                        {'type': 'condition', **node.condition}))
                    await sock.send_text(json.dumps(
                        {'type': 'pan', **node.pan}))
                    await asyncio.sleep(0.2)
            except Exception:                                # noqa: BLE001
                pass

        pusher = asyncio.create_task(push_target())
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get('type')

                if kind == 'teleop':
                    node.publish_teleop(msg.get('throttle', 0.0),
                                        msg.get('steering', 0.0))
                elif kind == 'video':
                    node.publish_video_config(msg.get('config', {}))
                elif kind == 'autonomy':
                    node.publish_autonomy(bool(msg.get('enabled', True)))
                elif kind == 'color':
                    node.publish_color_config(msg.get('config', {}))
                elif kind == 'arm':
                    node.publish_arm(bool(msg.get('armed', False)))
                elif kind == 'estop':
                    node.publish_estop(msg.get('engaged', True))
                elif kind == 'voice':
                    node.publish_voice(msg.get('text', ''),
                                       msg.get('source', 'phone-speech'),
                                       msg.get('alternatives'))
                elif kind == 'command':
                    node.publish_action(msg.get('action', ''),
                                        msg.get('seconds'))
                elif kind == 'pan':
                    node.publish_pan(msg.get('deg', 0.0))
                elif kind == 'lock':
                    node.publish_lock(bool(msg.get('engage', True)))
                elif kind == 'ping':
                    # Round-trip probe. The phone stamps and we echo, so the page
                    # can show real latency instead of us assuming the tailnet is
                    # fast enough to stream a joystick over.
                    await sock.send_text(json.dumps(
                        {'type': 'pong', 't': msg.get('t')}))
        except WebSocketDisconnect:
            pass
        except Exception as e:                       # noqa: BLE001
            node.get_logger().warn(f'websocket error: {e}')
        finally:
            pusher.cancel()
            node.client_count -= 1
            node.get_logger().info(f'client gone ({node.client_count} left)')
            if node.client_count <= 0:
                # Stop asserting teleop immediately: a vanished phone must not
                # keep the sticks alive. Disarming waits, because a dropped
                # socket usually means a sleeping screen, not a departed operator.
                node.publish_teleop(0.0, 0.0)
                node.schedule_disarm()

    return app


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()

    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    config = uvicorn.Config(build_app(node), host='0.0.0.0', port=node.port,
                            log_level='warning', loop='asyncio')
    server = uvicorn.Server(config)
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_teleop(0.0, 0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
