#!/usr/bin/env python3
"""Print actual detection boxes, to tell duplicates from real multiples.

Ten boxes per frame means very different things depending on where they are:
stacked on one body is a decoding or NMS fault, spread across the frame is a
busy room or a false-positive-prone model. This prints coordinates so the
question is answered by data.

  docker exec Doggobot python3 .../tools/probe_detections.py [conf]
"""
import os
import sys
import time

import depthai as dai

MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')
ARCHIVE = os.path.join(MODELS, 'person-yolo11n-416.tar.xz')
CONF = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb = cam.requestOutput((416, 416), dai.ImgFrame.Type.BGR888p,
                            dai.ImgResizeMode.LETTERBOX, 30)
    det = pipeline.create(dai.node.DetectionNetwork)
    det.build(rgb, dai.NNArchive(ARCHIVE), CONF)
    q = det.out.createOutputQueue()
    pipeline.start()

    print(f'conf={CONF}. Showing 6 frames of detections.\n')
    shown, t0 = 0, time.time()
    while shown < 6 and time.time() - t0 < 20:
        pkt = q.tryGet()
        if pkt is None:
            time.sleep(0.005)
            continue
        if not pkt.detections:
            continue
        shown += 1
        ds = sorted(pkt.detections, key=lambda d: -d.confidence)
        print(f'frame {shown}: {len(ds)} detections')
        for d in ds[:12]:
            cx, cy = (d.xmin + d.xmax) / 2, (d.ymin + d.ymax) / 2
            print(f'   conf {d.confidence:.2f}  centre ({cx:.2f},{cy:.2f})  '
                  f'size ({d.xmax-d.xmin:.2f}x{d.ymax-d.ymin:.2f})')
        print()
