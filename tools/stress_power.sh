#!/usr/bin/env bash
# Power stress test: perception running WHILE the motor accelerates hard.
#
#   docker exec Doggobot bash .../tools/stress_power.sh [seconds]
#
# CAR MUST BE ON A STAND.
#
# Why this test exists: the OAK-D and the LiDAR are powered from a DC-DC
# converter off the same 4S pack that feeds the VESC. Motor current transients
# sag that pack, and if the converter dips below dropout the USB peripherals
# reset. That failure cannot appear in a static bench test, only under motion,
# which is exactly how it would be discovered at a demo instead of in a lab.
#
# Watches three things: Pi undervoltage flags, USB resets in the kernel log, and
# whether perception's frame rate survives.

source "$(dirname "$0")/env.sh" >/dev/null 2>&1
SECS=${1:-60}
HERE="$(dirname "$0")"

echo "=== BEFORE ==="
echo "throttled: $(vcgencmd get_throttled 2>/dev/null || echo 'n/a in container')"
dmesg 2>/dev/null | tail -1 > /tmp/dmesg_mark || true

pubs=$(ros2 topic info /cmd_vel 2>/dev/null | grep -oP 'Publisher count: \K\d+')
if [ "$pubs" != "1" ]; then
  echo "WARNING: /cmd_vel has ${pubs:-0} publishers. Motor bursts will not run."
  DRIVE=0
else
  DRIVE=1
fi

echo
echo "=== starting perception for ${SECS}s ==="
TRACKER=color TRACKER_HOST=1 STEREO_PRESET=FAST_DENSITY \
  python3 "$HERE/probe_perception.py" "$SECS" > /tmp/stress_perception.log 2>&1 &
PERC=$!
sleep 6

if [ "$DRIVE" = "1" ]; then
  echo "=== hammering the motor: 8 accelerate/stop cycles ==="
  for i in $(seq 1 8); do
    echo "  burst $i: 0.37 forward"
    timeout 3 ros2 topic pub -r 10 /behavior_cmd geometry_msgs/msg/Twist \
      "{linear: {x: 0.37}, angular: {z: 0.0}}" >/dev/null 2>&1
    sleep 1
    echo "  burst $i: -0.37 reverse (worst case, direction reversal)"
    timeout 2 ros2 topic pub -r 10 /behavior_cmd geometry_msgs/msg/Twist \
      "{linear: {x: -0.37}, angular: {z: 0.0}}" >/dev/null 2>&1
    sleep 1
  done
else
  echo "(skipping motor bursts)"
fi

wait $PERC 2>/dev/null

echo
echo "=== perception result under load ==="
tail -5 /tmp/stress_perception.log

echo
echo "=== AFTER ==="
echo "throttled: $(vcgencmd get_throttled 2>/dev/null || echo 'n/a in container')"
echo "  0x0 = clean. bit0 undervoltage now, bit16 undervoltage HAS occurred."
echo
echo "=== USB / camera events in kernel log ==="
dmesg 2>/dev/null | grep -iE "usb|xhci|reset|Movidius|03e7" | tail -12 \
  || echo "(dmesg not readable from inside the container; run on the Pi host)"
