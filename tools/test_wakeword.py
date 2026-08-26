#!/usr/bin/env python3
"""Which candidate wake words does the Vosk model actually know?

A constrained grammar can only match words in the model's lexicon. A word it has
never heard of is silently unmatchable, so the wake word must be checked rather
than chosen for style. Vosk logs a warning per out-of-vocabulary word, so this
turns the log level up and watches for it.

  docker exec Doggobot python3 .../tools/test_wakeword.py
"""
import io
import json
import os
import sys
from contextlib import redirect_stderr

from vosk import KaldiRecognizer, Model, SetLogLevel

CANDIDATES = [
    'doggo', 'rex', 'rover', 'scout', 'ranger', 'robot', 'buddy',
    'champ', 'ace', 'echo', 'falcon', 'hunter', 'bandit', 'ghost',
    'jarvis', 'computer', 'shadow', 'tango', 'viper', 'cobra',
]

MODELS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'models')
model = Model(os.path.join(MODELS, 'vosk-small-en'))

print(f'{"word":12} in-vocabulary?')
known, unknown = [], []
for w in CANDIDATES:
    SetLogLevel(0)
    err = io.StringIO()
    with redirect_stderr(err):
        KaldiRecognizer(model, 16000, json.dumps([w]))
    SetLogLevel(-1)
    msg = err.getvalue().lower()
    missing = 'not present' in msg or 'oov' in msg or 'unknown word' in msg
    (unknown if missing else known).append(w)
    print(f'{w:12} {"NO - unusable" if missing else "yes"}')

print(f'\nusable: {", ".join(known)}')
if unknown:
    print(f'unusable: {", ".join(unknown)}')
