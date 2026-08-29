#!/usr/bin/env python3
"""HSV colour detection for the mission cues.

Not a neural net. The proposal's mission statement keys on "green" and "red"
markers, and a threshold in HSV is both cheaper and far easier to debug than
training a classifier for two flat colours.

Why HSV and not RGB: in RGB, "red" is a diagonal region that moves as brightness
changes, so a threshold tuned in one light fails in another. HSV separates hue
(which colour) from saturation (how vivid) and value (how bright), so a colour
occupies a compact box that survives moderate lighting change. It is not
lighting-invariant, but it degrades far more gracefully.

**Red wraps around the hue circle.** OpenCV maps hue to 0-179, and red sits at
BOTH ends: roughly 0-10 and 170-179. A single range cannot express it, so red
uses two masks OR'd together. This is the classic mistake in HSV colour work and
it presents as "red detection sort of works", which is worse than failing.

Thresholds are data, not code: they are loaded from and saved to JSON so they can
be retuned at the venue without a rebuild.
"""
import json
import os

import cv2
import numpy as np

# Deliberately wide starting points. Tuning narrows them; starting narrow means
# seeing nothing and having no idea which bound is wrong.
DEFAULTS = {
    'green': {
        'ranges': [[35, 60, 40, 85, 255, 255]],      # h_lo s_lo v_lo h_hi s_hi v_hi
        'min_area': 1500,
    },
    'red': {
        # Two ranges because hue wraps. Both must be tuned.
        'ranges': [[0, 90, 50, 10, 255, 255],
                   [170, 90, 50, 179, 255, 255]],
        'min_area': 1500,
    },
}


class ColorDetector:

    def __init__(self, path=None):
        self.path = path
        self.config = json.loads(json.dumps(DEFAULTS))   # deep copy
        self.load()

    # -- persistence ----------------------------------------------------------

    def load(self):
        if not self.path or not os.path.isfile(self.path):
            return False
        try:
            with open(self.path, encoding='utf-8') as f:
                loaded = json.load(f)
            for name, cfg in loaded.items():
                if name in self.config:
                    self.config[name].update(cfg)
            return True
        except Exception:                                # noqa: BLE001
            return False

    def save(self):
        if not self.path:
            return False
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, 'w', encoding='utf-8') as f:
                json.dump(self.config, f, indent=2)
            return True
        except Exception:                                # noqa: BLE001
            return False

    def update(self, name, ranges=None, min_area=None):
        if name not in self.config:
            return False
        if ranges is not None:
            self.config[name]['ranges'] = ranges
        if min_area is not None:
            self.config[name]['min_area'] = int(min_area)
        return True

    # -- detection ------------------------------------------------------------

    def mask_for(self, hsv, name):
        cfg = self.config[name]
        mask = None
        for r in cfg['ranges']:
            lo = np.array(r[0:3], dtype=np.uint8)
            hi = np.array(r[3:6], dtype=np.uint8)
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # Open then close: remove salt noise, then fill small holes so a textured
        # target counts as one blob rather than a hundred specks.
        k = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    def detect(self, frame):
        """Return (label, areas, masks).

        `label` is the colour with the largest qualifying blob, or None. Reporting
        the winner rather than every colour above threshold means a red marker
        partly in frame while approaching a green one cannot trigger both.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        areas, masks, best, best_area = {}, {}, None, 0

        for name in self.config:
            mask = self.mask_for(hsv, name)
            masks[name] = mask
            # Largest contour, not total pixel count: a wall speckled with
            # matching pixels should not outvote an actual object.
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            area = max((cv2.contourArea(c) for c in contours), default=0)
            areas[name] = int(area)
            if area >= self.config[name]['min_area'] and area > best_area:
                best, best_area = name, area

        return best, areas, masks

    def overlay(self, frame, masks, show=None):
        """Tint matching pixels so tuning is visible on the video feed."""
        out = frame
        tints = {'green': (0, 255, 0), 'red': (0, 0, 255)}
        for name, mask in masks.items():
            if show and name != show:
                continue
            tint = np.zeros_like(frame)
            tint[:] = tints.get(name, (255, 255, 255))
            out = np.where(mask[:, :, None].astype(bool),
                           cv2.addWeighted(out, 0.35, tint, 0.65, 0), out)
        return out.astype(np.uint8)
