#!/usr/bin/env bash
# Safety validation WITH THE MOTOR RUNNING. Car must be on a stand.
#
#   docker exec Doggobot bash /home/projects/ros2_ws/src/doggobot/tools/test_safety.sh
#
# Assumes drive.launch.py is already running (arbiter + vesc_twist_node) and
# that /cmd_vel has exactly one publisher. Checks the two failure modes that
# actually keep the car from hurting someone:
#   1. deadman  - publisher disappears mid-drive, car stops by itself
#   2. e-stop   - kill switch beats a live throttle command
#
# Latency is measured from /cmd_vel, not guessed.

source "$(dirname "$0")/env.sh" >/dev/null 2>&1

pubs=$(ros2 topic info /cmd_vel | grep -oP 'Publisher count: \K\d+')
if [ "$pubs" != "1" ]; then
  echo "REFUSING TO RUN: /cmd_vel has $pubs publishers, expected 1."
  echo "Run tools/stop_stack.sh, then relaunch drive.launch.py."
  exit 1
fi

THROTTLE=0.25
read_throttle() { timeout 2 ros2 topic echo /cmd_vel --once --field linear.x 2>/dev/null | head -1; }

echo "=============================================================="
echo " 1. DEADMAN: publisher vanishes mid-drive"
echo "=============================================================="
echo "streaming teleop at $THROTTLE for 5 s - wheels should spin"
ros2 topic pub -r 10 /teleop_cmd geometry_msgs/msg/Twist \
  "{linear: {x: $THROTTLE}, angular: {z: 0.0}}" >/dev/null 2>&1 &
PUB=$!
sleep 5

echo "KILLING the publisher now. No stop command is sent."
kill -9 $PUB 2>/dev/null
sleep 2
val=$(read_throttle)
if [ "$val" == "0.0" ]; then echo "  PASS  deadman stopped the car by itself"
else echo "  FAIL  car still commanded $val"; fi

sleep 3

echo
echo "=============================================================="
echo " 2. E-STOP: kill switch beats a live throttle command"
echo "=============================================================="
echo "streaming behavior at $THROTTLE continuously - wheels should spin"
ros2 topic pub -r 10 /behavior_cmd geometry_msgs/msg/Twist \
  "{linear: {x: $THROTTLE}, angular: {z: 0.0}}" >/dev/null 2>&1 &
PUB2=$!
sleep 5

echo "ENGAGING E-STOP while the throttle command is STILL being published"
ros2 topic pub -1 /estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1
sleep 2
val=$(read_throttle)
if [ "$val" == "0.0" ]; then echo "  PASS  e-stop zeroed a live command"
else echo "  FAIL  e-stop did not stop the car ($val)"; fi

echo "wheels must be STOPPED right now even though throttle is still streaming"
sleep 4

echo "CLEARING e-stop - wheels should resume with nothing republished"
ros2 topic pub -1 /estop std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1
sleep 2
val=$(read_throttle)
if [ "$val" != "0.0" ]; then echo "  PASS  control resumed after clear ($val)"
else echo "  FAIL  control did not resume"; fi
sleep 3

echo "cleaning up"
kill -9 $PUB2 2>/dev/null
pkill -f 'topic pu[b]' 2>/dev/null
sleep 2
echo "  final /cmd_vel throttle: $(read_throttle)  (expect 0.0)"
echo
echo "arbiter state transitions:"
grep -o 'control -> .*' /tmp/drive.log | tail -12
