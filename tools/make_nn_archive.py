#!/usr/bin/env python3
"""Wrap our v2-era .blob into a DepthAI v3 NN Archive.

Why this exists: the container runs depthai 3.x, whose DetectionNetwork takes
its YOLO decoding configuration from an NN Archive rather than from setters. Our
model was exported at tools.luxonis.com as a bare `.blob` plus a v2-style JSON.
Rather than re-export by hand through a browser, this reads the blob's actual
tensor names and shapes and generates a schema-valid archive around it.

An NN Archive is just a tar containing `config.json` and the model file.

    docker exec Doggobot python3 .../tools/make_nn_archive.py
"""
import json
import os
import shutil
import sys
import tarfile
import tempfile

import depthai as dai

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(os.path.dirname(HERE), 'models')
BLOB = os.path.join(MODELS, 'person-yolo11n_openvino_2022.1_6shave.blob')
META = os.path.join(MODELS, 'person-yolo11n.json')
OUT = os.path.join(MODELS, 'person-yolo11n-416.tar.xz')

DTYPE = {'U8F': 'uint8', 'FP16': 'float16', 'FP32': 'float32'}


def main():
    old = json.load(open(META))
    nn = old['nn_config']
    spec = nn['NN_specific_metadata']
    labels = old['mappings']['labels']
    w, h = (int(x) for x in nn['input_size'].split('x'))

    blob = dai.OpenVINO.Blob(BLOB)
    (in_name, in_t), = blob.networkInputs.items()
    # Sort outputs by stride so the parser sees them large-to-small, which is the
    # order the Luxonis YOLO exporter emits (52, 26, 13 for a 416 input).
    outs = sorted(blob.networkOutputs.items(), key=lambda kv: -kv[1].dims[0])

    cfg = {
        'config_version': '1.0',
        'model': {
            'metadata': {
                'name': 'person-yolo11n-416',
                'path': os.path.basename(BLOB),
                'precision': 'float16',
            },
            'inputs': [{
                'name': in_name,
                'dtype': DTYPE.get(str(in_t.dataType).split('.')[-1], 'uint8'),
                'input_type': 'image',
                'shape': [1, 3, h, w],
                'layout': 'NCHW',
                'preprocessing': {
                    'mean': [0, 0, 0],
                    'scale': [255, 255, 255],
                    'reverse_channels': False,
                    'interleaved_to_planar': False,
                },
            }],
            'outputs': [
                {'name': n,
                 'dtype': DTYPE.get(str(t.dataType).split('.')[-1], 'float16')}
                for n, t in outs
            ],
            'heads': [{
                'parser': 'YOLO',
                'metadata': {
                    'classes': labels,
                    'n_classes': int(spec['classes']),
                    'iou_threshold': float(spec['iou_threshold']),
                    'conf_threshold': float(spec['confidence_threshold']),
                    'max_det': 300,
                    'anchors': spec.get('anchors') or [],
                    'subtype': 'yolov6',
                },
                'outputs': [n for n, _ in outs],
            }],
        },
    }

    tmp = tempfile.mkdtemp()
    try:
        with open(os.path.join(tmp, 'config.json'), 'w') as f:
            json.dump(cfg, f, indent=2)
        shutil.copy(BLOB, tmp)
        with tarfile.open(OUT, 'w:xz') as tar:
            for name in ('config.json', os.path.basename(BLOB)):
                tar.add(os.path.join(tmp, name), arcname=name)
    finally:
        shutil.rmtree(tmp)

    print(f'wrote {OUT} ({os.path.getsize(OUT)/1e6:.1f} MB)')
    print(f'input {in_name} {list(in_t.dims)} -> outputs '
          f'{[n for n, _ in outs]}')

    archive = dai.NNArchive(OUT)
    print('NNArchive loaded OK:', archive.getConfig() is not None)
    return 0


if __name__ == '__main__':
    sys.exit(main())
