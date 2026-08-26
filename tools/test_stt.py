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

import time
for n in (3, 2, 1):
    print(f'  {n}...', flush=True)
    time.sleep(1)
print(f'>>> RECORDING {SECS}s - SPEAK NOW <<<', flush=True)
subprocess.run(['arecord', '-D', 'plughw:2,0', '-f', 'S16_LE', '-r', '16000',
                '-c', '1', '-d', SECS, WAV],
               check=True, stderr=subprocess.DEVNULL)

import audioop
w = wave.open(WAV)
raw = w.readframes(w.getnframes())
peak, rms = audioop.max(raw, 2), audioop.rms(raw, 2)
print(f'captured {len(raw)} bytes  peak={peak}  rms={rms}')
print('  rms under ~200 = effectively silence; a high peak with low rms is a '
      'click or thump, not speech')

from vosk import KaldiRecognizer, Model, SetLogLevel
SetLogLevel(-1)
model = Model(MODEL)

for label, rec in (
        ('grammar-constrained', KaldiRecognizer(model, 16000,
                                                json.dumps(PHRASES + ['[unk]']))),
        ('free-form', KaldiRecognizer(model, 16000))):
    # Vosk expects a stream of chunks, not one enormous buffer.
    partials = []
    for i in range(0, len(raw), 4000):
        if rec.AcceptWaveform(raw[i:i + 4000]):
            t = json.loads(rec.Result()).get('text', '')
            if t:
                partials.append(t)
    t = json.loads(rec.FinalResult()).get('text', '')
    if t:
        partials.append(t)
    print(f'  {label:20} -> {" | ".join(partials)!r}')
