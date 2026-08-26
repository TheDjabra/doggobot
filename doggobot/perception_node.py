#!/usr/bin/env python3
"""Perception: camera -> a single locked target, published as state.

Publishes
    /target_state   std_msgs/String, JSON, at the pipeline rate (~12 Hz)

Subscribes
    /target_lock    std_msgs/String, JSON: {"action": "lock"} | {"action": "release"}

The message is deliberately everything a controller needs and nothing else:

    {"locked": true, "id": 3, "status": "TRACKED",
     "x": -0.21,          # bbox centre offset from frame centre, -1..1 (steering error)
     "y":  0.05,
     "z_mm": 1420,        # median-filtered depth (throttle error against a setpoint)
     "conf": 0.94, "age": 37, "fps": 12.3, "stamp": 1787...}

`x` is already an error term rather than a coordinate: 0 is centred, negative is
left. That keeps the follow controller a PD loop over two numbers and stops
frame geometry leaking into it.

Design decisions worth knowing:

**Lock-on, not identity.** "Follow" does not mean recognising a specific person,
which is face re-identification and far more fragile outdoors. On {"action":
"lock"} the node picks the highest-confidence tracklet currently in frame and
follows THAT tracklet id, ignoring everyone else until released or until the
tracker declares it REMOVED. This is the same click-to-lock pattern the AI
TURRET already uses.

**The tracker runs on the host.** On an OAK-D Lite already running a network and
stereo, the on-device ObjectTracker is allocated one SHAVE core and collapses the
pipeline from 7.6 fps to 1.2. `setRunOnHost(True)` restores it to ~12. Measured,
see docs/hardware.md.

**Depth is median-filtered over a short window.** Stereo Z is noisy frame to
frame, and the stereo median filter had to be disabled on-device because it
exhausts SIPP memory once a NN shares the chip.

**The camera is allowed to disappear.** The OAK-D re-enumerates on the USB bus
when a pipeline starts and stops, and a brownout would do the same. The pipeline
is rebuilt in a loop rather than taking the node down with it.
"""
import json
import os
import statistics
import threading
import time
from collections import deque

import depthai as dai
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        default_archive = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'models', 'person-yolo11n-416.tar.xz')

        self.declare_parameter('archive_path', default_archive)
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('nn_fps', 20.0)
        self.declare_parameter('depth_window', 5)
        self.declare_parameter('lock_on_start', False)
        self.declare_parameter('max_objects', 10)

        self.archive_path = self.get_parameter('archive_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.nn_fps = float(self.get_parameter('nn_fps').value)
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.max_objects = int(self.get_parameter('max_objects').value)

        self.locked_id = None
        self.want_lock = bool(self.get_parameter('lock_on_start').value)
        self.depths = deque(maxlen=self.depth_window)

        self.state_pub = self.create_publisher(String, 'target_state', 10)
        self.create_subscription(String, 'target_lock', self._on_lock, 10)

        self.running = True
        self.get_logger().info(f'perception: {os.path.basename(self.archive_path)} '
                               f'conf {self.confidence} @ {self.nn_fps:.0f} fps req')

    # -- lock control ---------------------------------------------------------

    def _on_lock(self, msg):
        try:
            action = json.loads(msg.data).get('action')
        except Exception:                                    # noqa: BLE001
            self.get_logger().warn(f'unparseable target_lock: {msg.data!r}')
            return
        if action == 'lock':
            self.want_lock = True
            self.get_logger().info('lock requested')
        elif action == 'release':
            self.want_lock = False
            self.locked_id = None
            self.depths.clear()
            self.get_logger().info('lock released')

    # -- publishing -----------------------------------------------------------

    def _publish(self, payload):
        self.state_pub.publish(String(data=json.dumps(payload)))

    def _publish_no_target(self, fps, reason):
        self._publish({'locked': False, 'id': None, 'status': reason,
                       'x': 0.0, 'y': 0.0, 'z_mm': 0, 'conf': 0.0,
                       'age': 0, 'fps': round(fps, 1), 'stamp': time.time()})

    # -- the pipeline ---------------------------------------------------------

    def _build(self, pipeline):
        cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
        stereo = pipeline.create(dai.node.StereoDepth).build(
            autoCreateCameras=True,
            presetMode=dai.node.StereoDepth.PresetMode.FAST_DENSITY)
        # Off, or stereo halts with "'Median' out of system resources: '126'"
        # once the network shares the chip. We median-filter Z host-side instead.
        stereo.initialConfig.postProcessing.median = dai.MedianFilter.MEDIAN_OFF

        det = pipeline.create(dai.node.SpatialDetectionNetwork)
        det.build(cam, stereo, dai.NNArchive(self.archive_path), fps=self.nn_fps)
        det.setConfidenceThreshold(self.confidence)

        tracker = pipeline.create(dai.node.ObjectTracker)
        tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        # A person who leaves and returns gets a NEW id, so "did I lose my
        # target" is never ambiguous and the pan sweep has a clean trigger.
        tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
        tracker.setMaxObjectsToTrack(self.max_objects)
        tracker.setRunOnHost(True)

        det.passthrough.link(tracker.inputTrackerFrame)
        det.passthrough.link(tracker.inputDetectionFrame)
        det.out.link(tracker.inputDetections)
        return tracker.out.createOutputQueue()

    def _select(self, tracklets):
        """Return the tracklet we are following, acquiring a lock if asked."""
        live = [t for t in tracklets
                if str(t.status).split('.')[-1] in ('TRACKED', 'NEW')]

        if self.locked_id is not None:
            for t in tracklets:
                if t.id == self.locked_id:
                    if str(t.status).split('.')[-1] == 'REMOVED':
                        self.get_logger().info(f'target {self.locked_id} REMOVED')
                        self.locked_id = None
                        self.depths.clear()
                        return None
                    return t
            return None                       # id vanished entirely

        if self.want_lock and live:
            best = max(live, key=lambda t: t.srcImgDetection.confidence)
            self.locked_id = best.id
            self.depths.clear()
            self.get_logger().info(
                f'locked onto id {best.id} '
                f'(conf {best.srcImgDetection.confidence:.2f})')
            return best
        return None

    def spin_camera(self):
        """Run the pipeline, rebuilding it if the camera goes away."""
        while self.running and rclpy.ok():
            try:
                with dai.Pipeline() as pipeline:
                    q = self._build(pipeline)
                    pipeline.start()
                    self.get_logger().info('camera pipeline started')
                    self._loop(q)
            except Exception as e:                            # noqa: BLE001
                if not self.running:
                    return
                self.get_logger().error(f'camera pipeline died: {e}')
                self._publish_no_target(0.0, 'CAMERA_DOWN')
                time.sleep(2.0)

    def _loop(self, q):
        frames, t0, fps = 0, time.time(), 0.0
        while self.running and rclpy.ok():
            pkt = q.tryGet()
            if pkt is None:
                time.sleep(0.002)
                continue

            frames += 1
            if frames % 20 == 0:
                now = time.time()
                fps = 20.0 / (now - t0)
                t0 = now

            t = self._select(pkt.tracklets)
            if t is None:
                self._publish_no_target(fps, 'NO_TARGET')
                continue

            status = str(t.status).split('.')[-1]
            roi = t.roi
            z = t.spatialCoordinates.z
            if status == 'TRACKED' and z > 0:
                self.depths.append(z)
            z_med = statistics.median(self.depths) if self.depths else 0

            self._publish({
                'locked': True,
                'id': t.id,
                'status': status,
                # centre offset, -1..1, so this is already the steering error
                'x': round((roi.x + roi.width / 2) * 2.0 - 1.0, 4),
                'y': round((roi.y + roi.height / 2) * 2.0 - 1.0, 4),
                'z_mm': int(z_med),
                'conf': round(t.srcImgDetection.confidence, 3),
                'age': t.age,
                'fps': round(fps, 1),
                'stamp': time.time(),
            })


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()
    try:
        node.spin_camera()
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
