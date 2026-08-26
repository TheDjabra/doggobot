#!/usr/bin/env python3
"""The full perception pipeline: stereo depth + detection + tracking.

This is the shape perception_node will take, run as a throwaway so the costs are
measured before any ROS2 code depends on them. Three questions it answers:

  1. What does stacking StereoDepth and ObjectTracker do to the 23 fps the
     detector managed on its own?
  2. Does spatialCoordinates give sane distances, and how noisy are they?
  3. Does the OAK-D hold its power budget with RGB + stereo + NN + tracker all
     running? This matters more now that the LiDAR shares the USB bus.

Tracklet status is the lock-on mechanism: follow while TRACKED, coast on LOST,
start the pan sweep on REMOVED.

  docker exec Doggobot python3 .../tools/probe_perception.py [seconds]
"""
import os
import sys
import time
from collections import Counter

import depthai as dai

MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')
ARCHIVE = os.path.join(MODELS, os.environ.get('ARCHIVE_NAME', 'person-yolo11n-416.tar.xz'))
SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
CONF = float(os.environ.get('CONF', '0.5'))

archive = dai.NNArchive(ARCHIVE)

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    stereo = pipeline.create(dai.node.StereoDepth).build(
        autoCreateCameras=True,
        presetMode=dai.node.StereoDepth.PresetMode.ROBOTICS)
    # The OAK-D Lite runs out of SIPP buffer memory with the median filter on
    # once a NN and tracker share the chip: "'Median' out of system resources:
    # '126'". Turning it off is the cheap fix; depth gets slightly noisier, which
    # the median filter we apply host-side on Z compensates for anyway.
    stereo.initialConfig.postProcessing.median = dai.MedianFilter.MEDIAN_OFF

    det = pipeline.create(dai.node.SpatialDetectionNetwork)
    # The fps argument is NOT optional in practice. Without it the node requests
    # camera output at a default that collapses the pipeline to about 1 fps.
    det.build(cam, stereo, archive, fps=float(os.environ.get('NN_FPS', '20')))
    det.setConfidenceThreshold(CONF)

    TRACKER = os.environ.get('TRACKER', 'color')   # color | imageless | none

    if TRACKER == 'none':
        q = det.out.createOutputQueue()
        pipeline.start()
        print(f'archive={os.path.basename(ARCHIVE)} conf={CONF} TRACKER=none  '
              f'sampling {SECONDS:.0f} s')
        t0, n, zs = time.time(), 0, []
        while time.time() - t0 < SECONDS:
            pkt = q.tryGet()
            if pkt is None:
                time.sleep(0.002); continue
            n += 1
            for d in pkt.detections:
                zs.append(d.spatialCoordinates.z)
        dt = time.time() - t0
        print(f'\nRESULT: {n} frames in {dt:.1f} s = {n/dt:.1f} fps  '
              f'(spatial detection only, no tracker)')
        if zs:
            print(f'        depth {min(zs):.0f} to {max(zs):.0f} mm '
                  f'over {len(zs)} detections')
        raise SystemExit(0)

    tracker = pipeline.create(dai.node.ObjectTracker)
    # Colour histogram helps re-associate a target after brief occlusion, which
    # is the whole point of lock-on. UNIQUE_ID means a person who leaves and
    # returns gets a NEW id, so "did I lose my target" stays unambiguous.
    tracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM
                           if TRACKER == 'color'
                           else dai.TrackerType.ZERO_TERM_IMAGELESS)
    tracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.UNIQUE_ID)
    tracker.setMaxObjectsToTrack(10)

    det.passthrough.link(tracker.inputTrackerFrame)
    det.passthrough.link(tracker.inputDetectionFrame)
    det.out.link(tracker.inputDetections)

    q = tracker.out.createOutputQueue()
    pipeline.start()
    print(f'archive={os.path.basename(ARCHIVE)} conf={CONF}  '
          f'sampling {SECONDS:.0f} s')
    print('walk toward and away from the camera, then step out of frame\n')

    t0 = time.time()
    frames = 0
    statuses = Counter()
    ids = set()
    last_print = 0.0
    zmin, zmax = 1e9, -1e9

    while time.time() - t0 < SECONDS:
        pkt = q.tryGet()
        if pkt is None:
            time.sleep(0.002)
            continue
        frames += 1
        now = time.time() - t0
        for t in pkt.tracklets:
            st = str(t.status).split('.')[-1]
            statuses[st] += 1
            ids.add(t.id)
            z = t.spatialCoordinates.z
            if st == 'TRACKED' and z > 0:
                zmin, zmax = min(zmin, z), max(zmax, z)
            if now - last_print > 1.0:
                last_print = now
                roi = t.roi
                print(f'  t={now:4.1f}s id={t.id:<3} {st:<8} '
                      f'x={roi.x + roi.width / 2:.2f} '
                      f'XYZ=({t.spatialCoordinates.x:7.0f},'
                      f'{t.spatialCoordinates.y:7.0f},{z:7.0f}) mm')

    dt = time.time() - t0
    print(f'\nRESULT: {frames} tracker frames in {dt:.1f} s = {frames/dt:.1f} fps')
    print(f'        statuses: {dict(statuses)}')
    print(f'        distinct tracklet ids: {sorted(ids)}')
    if zmax > 0:
        print(f'        depth range while TRACKED: {zmin:.0f} to {zmax:.0f} mm')
