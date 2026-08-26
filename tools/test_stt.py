#!/usr/bin/env python3
"""Record from the mic and decode it, with ROS out of the picture.

Separates three things that all present as "the car does not hear me": the
microphone not capturing, Vosk not decoding, or the ROS node not publishing.

  docker exec Doggobot python3 .../tools/test_stt.py [seconds]
"""
import json
import os
import subprocess
import sys
import wave

MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')
MODEL = os.path.join(MODELS, 'vosk-small-en')
WAV = '/tmp/stt_probe.wav'
SECS = sys.argv[1] if len(sys.argv) > 1 else '5'

PHRASES = ['stop', 'halt', 'wait', 'hold', 'stay', 'freeze',
           'forward', 'go forward', 'go straight', 'ahead',
           'reverse', 'back up', 'go back', 'backward', 'backwards',
           'circle right', 'circle to the right',
           'circle left', 'circle to the left',
           'follow', 'follow me', 'come here']

print(f'recording {SECS}s from plughw:2,0 - SPEAK NOW')
subprocess.run(['arecord', '-D', 'plughw:2,0', '-f', 'S16_LE', '-r', '16000',
                '-c', '1', '-d', SECS, WAV],
               check=True, stderr=subprocess.DEVNULL)

import audioop
w = wave.open(WAV)
raw = w.readframes(w.getnframes())
print(f'captured {len(raw)} bytes, peak amplitude {audioop.max(raw, 2)} '
      f'(under ~500 means effectively silence)')

from vosk import KaldiRecognizer, Model, SetLogLevel
SetLogLevel(-1)
model = Model(MODEL)

for label, rec in (
        ('grammar-constrained', KaldiRecognizer(model, 16000,
                                                json.dumps(PHRASES + ['[unk]']))),
        ('free-form', KaldiRecognizer(model, 16000))):
    rec.AcceptWaveform(raw)
    text = json.loads(rec.FinalResult()).get('text', '')
    print(f'  {label:20} -> {text!r}')
