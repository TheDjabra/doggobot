#!/usr/bin/env bash
# Make the car start its own stack at power-on, so the phone app just works.
#
# Run on the Pi HOST (not in the container):
#     bash tools/install_service.sh
#
# Afterwards:
#     sudo systemctl status doggobot      what it is doing
#     journalctl -u doggobot -f           live logs
#     sudo systemctl stop doggobot        take manual control back
#     sudo systemctl disable doggobot     stop it starting at boot
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "installing unit..."
sudo cp "$HERE/../deploy/doggobot.service" /etc/systemd/system/doggobot.service

# Docker will bring the container back after a reboot on its own; the unit then
# only has to launch the stack inside it.
echo "setting container restart policy..."
docker update --restart unless-stopped Doggobot >/dev/null

echo "enabling..."
sudo systemctl daemon-reload
sudo systemctl enable doggobot
sudo systemctl restart doggobot

sleep 12
sudo systemctl --no-pager status doggobot | head -12
echo
echo "the app will be live at https://doggobot.tail502ca5.ts.net once nodes are up"
