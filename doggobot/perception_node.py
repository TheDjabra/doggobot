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

**Video is published only when watched.** JPEG encoding costs Pi CPU, so frames
are made only if something is subscribed and only at `video_fps`, well below the
pipeline rate. The frame comes from the tracker's own passthrough, so the overlay
is drawn on exactly the frame the tracklets describe.

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

import cv2
import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from doggobot.color import ColorDetector


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
        # After the tracker reports REMOVED, should we grab the next best target
        # automatically? Default NO. Losing a target is a state the operator
        # should be told about, not one the robot silently resolves by locking
        # onto whoever walks past. Left true, stepping out of frame and back gets
        # you a different person with no announcement.
        self.declare_parameter('relock_on_loss', False)

        # Video for the phone. JPEG encoding costs Pi CPU that the control loop
        # needs more, so it runs well below the pipeline rate and only when
        # something is actually subscribed: nobody watching costs nothing.
        self.declare_parameter('video_fps', 8.0)
        self.declare_parameter('video_quality', 55)
        self.declare_parameter('video_annotate', True)
        # NOTE (2026-08-29): video is capped at the DETECTOR's rate, ~12 fps,
        # because frames come from the tracker's passthrough. Asking for 30 gives
        # 12.4. Adding a second `cam.requestOutput()` to bypass the NN was tried
        # and BROKE THE PIPELINE ENTIRELY: it constructed and reported "started"
        # while producing no tracker output at all. Reverted. Getting full-rate
        # video needs a different approach than a second output on this camera,
        # and is not worth risking a working perception path over.

        # Colour cues for the mission statement. Runs on the same frames the
        # video already uses, because only one process can own the camera.
        self.declare_parameter('color_enabled', True)
        self.declare_parameter('color_config_path',
                               '/home/projects/ros2_ws/src/doggobot/config/color_thresholds.json')
        self.declare_parameter('color_hz', 5.0)
        self.declare_parameter('color_view', '')   # '', 'green', 'red', 'all'

        # Lock the camera's exposure and white balance. OFF by default because
        # auto is better for detection in changing light; ON is for colour work.
        #
        # Auto white balance actively changes how colour is rendered as the scene
        # shifts, which means HSV thresholds are chasing a moving target: the same
        # object reads as a different hue depending on what else is in frame. In a
        # room mixing 4000 K room lighting with 5500-6500 K daylight, that is the
        # difference between thresholds that hold and thresholds that drift.
        # Lock them, THEN tune, and do both at the venue.
        self.declare_parameter('lock_camera', False)
        self.declare_parameter('exposure_us', 8000)
        self.declare_parameter('iso', 400)
        self.declare_parameter('white_balance_k', 5000)

        self.archive_path = self.get_parameter('archive_path').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.nn_fps = float(self.get_parameter('nn_fps').value)
        self.depth_window = int(self.get_parameter('depth_window').value)
        self.max_objects = int(self.get_parameter('max_objects').value)
        self.relock_on_loss = bool(self.get_parameter('relock_on_loss').value)
        self.video_fps = float(self.get_parameter('video_fps').value)
        self.video_quality = int(self.get_parameter('video_quality').value)
        self.video_annotate = bool(self.get_parameter('video_annotate').value)

        self._last_frame = 0.0

        self.color_enabled = bool(self.get_parameter('color_enabled').value)
        self.color_hz = float(self.get_parameter('color_hz').value)
        self.color_view = str(self.get_parameter('color_view').value)
        self.lock_camera = bool(self.get_parameter('lock_camera').value)
        self.exposure_us = int(self.get_parameter('exposure_us').value)
        self.iso = int(self.get_parameter('iso').value)
        self.wb_k = int(self.get_parameter('white_balance_k').value)
        self.colors = ColorDetector(str(self.get_parameter('color_config_path').value))
        self._last_color = 0.0
        self._masks = {}
        self._color_state = {'color': None, 'areas': {}}

        self.locked_id = None
        self.want_lock = bool(self.get_parameter('lock_on_start').value)
        self.depths = deque(maxlen=self.depth_window)

        self.state_pub = self.create_publisher(String, 'target_state', 10)
        self.image_pub = self.create_publisher(CompressedImage, 'camera/compressed', 2)
        self.condition_pub = self.create_publisher(String, 'condition_state', 10)
        self.create_subscription(String, 'color_config', self._on_color_config, 10)
        self.create_subscription(String, 'video_config', self._on_video_config, 10)
        self.image_pub = self.create_publisher(CompressedImage, 'camera/compressed', 2)
        self.condition_pub = self.create_publisher(String, 'condition_state', 10)
        self.create_subscription(String, 'color_config', self._on_color_config, 10)
        self.create_subscription(String, 'video_config', self._on_video_config, 10)
        self.create_subscription(String, 'target_lock', self._on_lock, 10)

        self.running = True
        self.get_logger().info(f'perception: {os.path.basename(self.archive_path)} '
                               f'conf {self.confidence} @ {self.nn_fps:.0f} fps req')

    # -- colour ---------------------------------------------------------------

    def _on_video_config(self, msg):
        """Runtime video rate, so the manual tab can ask for more frames.

        Measured: 8 fps of JPEG costs about 4.6% of the container. Raising it is
        affordable, and being able to drop it without a redeploy matters if the
        link is poor at the venue.
        """
        try:
            m = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return
        if 'fps' in m:
            self.video_fps = max(0.0, min(30.0, float(m['fps'])))
        if 'quality' in m:
            self.video_quality = max(20, min(90, int(m['quality'])))

        self.get_logger().info(
            f'video {self.video_fps:.0f} fps q{self.video_quality}')

    def _on_color_config(self, msg):
        """Live threshold updates from the tuning page.

        Tuning HSV by editing a file and restarting is unusable: you need to see
        the mask change as you drag a slider. Thresholds are therefore data, sent
        over a topic and optionally persisted.
        """
        try:
            m = json.loads(msg.data)
        except Exception:                                # noqa: BLE001
            return
        if m.get('view') is not None:
            self.color_view = m['view']
        for name in ('green', 'red'):
            if name in m:
                self.colors.update(name, m[name].get('ranges'),
                                   m[name].get('min_area'))
        if m.get('save'):
            ok = self.colors.save()
            self.get_logger().info(
                f'colour thresholds {"saved" if ok else "NOT saved"}')

    def _run_color(self, frame):
        self._last_color = time.time()
        label, areas, masks = self.colors.detect(frame)
        self._masks = masks
        self._color_state = {'color': label, 'areas': areas}
        # Published continuously, including when nothing is seen, so a condition
        # that stops being true is observable rather than merely stale.
        self.condition_pub.publish(String(data=json.dumps(
            {'color': label, 'areas': areas, 'stamp': self._last_color})))

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
        if self.lock_camera:
            cam.initialControl.setManualExposure(self.exposure_us, self.iso)
            cam.initialControl.setManualWhiteBalance(self.wb_k)
            self.get_logger().info(
                f'camera LOCKED: {self.exposure_us} us, ISO {self.iso}, '
                f'{self.wb_k} K')
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
        # The tracker passes its input frame through, so the video comes from the
        # same packet as the tracklets. No second camera stream, and the overlay
        # is guaranteed to match the frame it is drawn on.
        return (tracker.out.createOutputQueue(),
                tracker.passthroughTrackerFrame.createOutputQueue())

    def _select(self, tracklets):
        """Return the tracklet we are following, acquiring a lock if asked."""
        live = [t for t in tracklets
                if str(t.status).split('.')[-1] in ('TRACKED', 'NEW')]

        if self.locked_id is not None:
            for t in tracklets:
                if t.id == self.locked_id:
                    if str(t.status).split('.')[-1] == 'REMOVED':
                        self.get_logger().warn(
                            f'target {self.locked_id} REMOVED'
                            + ('' if self.relock_on_loss else ', lock dropped'))
                        self.locked_id = None
                        self.depths.clear()
                        if not self.relock_on_loss:
                            self.want_lock = False
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
                    q, qframe = self._build(pipeline)
                    pipeline.start()
                    self.get_logger().info('camera pipeline started')
                    self._loop(q, qframe)
            except Exception as e:                            # noqa: BLE001
                if not self.running:
                    return
                self.get_logger().error(f'camera pipeline died: {e}')
                self._publish_no_target(0.0, 'CAMERA_DOWN')
                # The device does not release immediately after a crash; retrying
                # too fast just collides with the dying pipeline and reports
                # ALREADY_IN_USE indefinitely, which hides the real error.
                time.sleep(6.0)

    def _on_frame(self, qframe, tracklets):
        """One frame, two consumers: colour detection and (optionally) video.

        Both want the same picture, and colour must see the RAW frame rather than
        one with an overlay already drawn on it, so the order here matters:
        fetch once, detect, then annotate a copy for the phone.
        """
        want_video = (self.video_fps > 0
                      and self.image_pub.get_subscription_count() > 0)
        now = time.time()
        video_due = want_video and (now - self._last_frame >= 1.0 / self.video_fps)
        color_due = self.color_enabled and (
            now - self._last_color >= 1.0 / self.color_hz)

        if not (video_due or color_due):
            qframe.tryGet()          # drain, or the queue backs up
            return

        pkt = qframe.tryGet()
        if pkt is None:
            return
        frame = pkt.getCvFrame()

        if color_due:
            self._run_color(frame)

        if not video_due:
            return
        self._last_frame = now

        annotate = self.video_annotate
        if self.color_view and self._masks:
            show = None if self.color_view == 'all' else self.color_view
            frame = self.colors.overlay(frame, self._masks, show)

        if annotate:
            h, w = frame.shape[:2]
            for t in tracklets:
                if str(t.status).split('.')[-1] not in ('TRACKED', 'NEW', 'LOST'):
                    continue
                mine = (t.id == self.locked_id)
                roi = t.roi.denormalize(w, h)
                colour = (0, 176, 255) if mine else (110, 110, 110)
                cv2.rectangle(frame, (int(roi.x), int(roi.y)),
                              (int(roi.x + roi.width), int(roi.y + roi.height)),
                              colour, 2 if mine else 1)
                if mine:
                    cv2.putText(frame,
                                f'{t.spatialCoordinates.z / 1000:.2f}m',
                                (int(roi.x), max(14, int(roi.y) - 6)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1)

        ok, jpg = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.video_quality])
        if ok:
            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = jpg.tobytes()
            self.image_pub.publish(msg)

    def _loop(self, q, qframe):
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

            self._on_frame(qframe, pkt.tracklets)

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
