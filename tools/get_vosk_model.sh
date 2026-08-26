#!/usr/bin/env bash
# Fetch the Vosk speech model into the bind-mounted models directory.
#
# It has to live under /home/pi/doggobot (which the container mounts at
# /home/projects/ros2_ws/src/doggobot) rather than /home/pi/models, or the
# container cannot see it. Gitignored: 68 MB of binary does not belong in git.
#
#   bash tools/get_vosk_model.sh          # run on the Pi HOST, not in the container
set -e
DEST="$(cd "$(dirname "$0")/.." && pwd)/models/vosk-small-en"
if [ -d "$DEST" ]; then echo "already present: $DEST"; exit 0; fi
URL=https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip
TMP=$(mktemp -d)
echo "downloading..."
curl -sL -o "$TMP/v.zip" "$URL"
unzip -q "$TMP/v.zip" -d "$TMP"
mkdir -p "$(dirname "$DEST")"
mv "$TMP"/vosk-model-small-en-us-* "$DEST"
rm -rf "$TMP"
echo "installed: $DEST ($(du -sh "$DEST" | cut -f1))"
