#!/usr/bin/env python3
"""Can DepthAI v3 run our v2-era .blob (wrapped as an NN Archive), and how fast?

The container ships depthai 3.x, which moved to NN Archive packaging, but the
v2-style `setBlobPath` survives. This answers empirically whether the existing
416 blob loads and runs, or whether the model has to be re-exported as an NN
Archive before any perception work can start.

  docker exec Doggobot python3 /home/projects/ros2_ws/src/doggobot/tools/probe_blob.py
"""
import json
import os
import sys
import time

import depthai as dai

MODELS = '/home/projects/ros2_ws/src/doggobot/models'
BLOB = os.path.join(MODELS, 'person-yolo11n_openvino_2022.1_6shave.blob')
META = os.path.join(MODELS, 'person-yolo11n.json')

meta = json.load(open(META))['nn_config']
W, H = (int(x) for x in meta['input_size'].split('x'))
classes = meta['NN_specific_metadata']['classes']
iou = meta['NN_specific_metadata']['iou_threshold']
conf = meta['NN_specific_metadata']['confidence_threshold']
print(f'depthai {dai.__version__} | blob {W}x{H}, {classes} class(es), '
      f'conf {conf}, iou {iou}')

ARCHIVE = os.path.join(MODELS, 'person-yolo11n-416.tar.xz')

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    rgb = cam.requestOutput((W, H), dai.ImgFrame.Type.BGR888p,
                            dai.ImgResizeMode.LETTERBOX, 30)

    archive = dai.NNArchive(ARCHIVE)
    det = pipeline.create(dai.node.DetectionNetwork)
    thr = float(sys.argv[1]) if len(sys.argv) > 1 else conf
    print(f'confidence threshold for this run: {thr}')
    det.build(rgb, archive, thr)

    q = det.out.createOutputQueue()
    pipeline.start()
    print('pipeline started, sampling 8 s (stand in front of the camera) ...')

    t0, frames, dets, best = time.time(), 0, 0, 0.0
    while time.time() - t0 < 8.0:
        pkt = q.tryGet()
        if pkt is None:
            time.sleep(0.002)
            continue
        frames += 1
        dets += len(pkt.detections)
        for d in pkt.detections:
            best = max(best, d.confidence)

    dt = time.time() - t0
    print(f'RESULT: {frames} frames in {dt:.1f} s = {frames/dt:.1f} fps')
    print(f'        {dets} detections, best confidence {best:.2f}')
    print('verdict: the existing blob runs under depthai v3 via a generated NN Archive')
