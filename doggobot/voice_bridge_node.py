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
from fastapi.responses import FileResponse, HTMLResponse
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, String

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
        self.declare_parameter('throttle_floor', 0.13)

        self.port = int(self.get_parameter('port').value)
        self.max_teleop_throttle = float(
            self.get_parameter('max_teleop_throttle').value)
        self.throttle_floor = float(self.get_parameter('throttle_floor').value)

        self.teleop_pub = self.create_publisher(Twist, 'teleop_cmd', 10)
        self.estop_pub = self.create_publisher(Bool, 'estop', 10)
        self.voice_pub = self.create_publisher(String, 'voice_cmd', 10)

        # NOT `self.clients`: rclpy.node.Node already defines `clients` as a
        # read-only property listing this node's service clients.
        self.client_count = 0
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

    def publish_voice(self, text, source):
        self.voice_pub.publish(String(data=json.dumps(
            {'text': text, 'source': source})))
        self.get_logger().info(f'voice[{source}]: {text}')


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

    @app.get('/healthz')
    async def healthz():
        return {'ok': True, 'clients': node.client_count}

    @app.websocket('/ws')
    async def ws(sock: WebSocket):
        await sock.accept()
        node.client_count += 1
        node.get_logger().info(f'client connected ({node.client_count} total)')
        try:
            while True:
                msg = json.loads(await sock.receive_text())
                kind = msg.get('type')

                if kind == 'teleop':
                    node.publish_teleop(msg.get('throttle', 0.0),
                                        msg.get('steering', 0.0))
                elif kind == 'estop':
                    node.publish_estop(msg.get('engaged', True))
                elif kind == 'voice':
                    node.publish_voice(msg.get('text', ''),
                                       msg.get('source', 'speech'))
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
            node.client_count -= 1
            node.get_logger().info(f'client gone ({node.client_count} left)')
            if node.client_count <= 0:
                # Last client out: stop asserting teleop. The arbiter's staleness
                # timeout does the rest.
                node.publish_teleop(0.0, 0.0)

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
