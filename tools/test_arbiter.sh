#!/usr/bin/env bash
# Exercise arbiter_node's decision logic with no actuator attached.
#
# Run INSIDE the Doggobot container:
#   docker exec -it Doggobot bash /home/projects/ros2_ws/src/doggobot/tools/test_arbiter.sh
#
# Nothing here starts the VESC node, so the car cannot move. This checks the
# priority order, the staleness timeouts, the e-stop, and the output clamp.

source "$(dirname "$0")/env.sh" >/dev/null 2>&1

pass=0; fail=0
check() {  # check <label> <expected-substring> <actual>
  if [[ "$3" == *"$2"* ]]; then echo "  PASS  $1"; pass=$((pass+1))
  else echo "  FAIL  $1"; echo "        expected to contain: $2"; echo "        got: $3"; fail=$((fail+1)); fi
}

# One line of /cmd_vel as "throttle,steering", so tests read as numbers.
sample() {
  timeout 4 ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once 2>/dev/null \
    | python3 -c "
import sys,re
t=sys.stdin.read()
def g(block,axis):
    m=re.search(block+r':\s*\n\s*x:\s*([-\d.e]+)\s*\n\s*y:\s*([-\d.e]+)\s*\n\s*z:\s*([-\d.e]+)',t)
    return float(m.group({'x':1,'y':2,'z':3}[axis])) if m else None
print(f'{g(\"linear\",\"x\"):.3f},{g(\"angular\",\"z\"):.3f}')
" 2>/dev/null || echo "NO_DATA"
}

pub_bg() {  # pub_bg <topic> <yaml>  -> streams at 10 Hz until killed
  ros2 topic pub -r 10 "$1" geometry_msgs/msg/Twist "$2" >/dev/null 2>&1 &
  echo $!
}

# Preflight: a stale arbiter from a previous run would make TWO publishers on
# /cmd_vel and silently corrupt every result below. Refuse to start instead.
if pgrep -f 'lib/doggobot/arbiter_node' >/dev/null 2>&1; then
  echo "REFUSING TO RUN: an arbiter_node is already alive."
  pgrep -af 'lib/doggobot/arbiter_node'
  echo "Stop it first:  pkill -f 'lib/doggobot/arbiter_node'"
  exit 1
fi

echo "starting arbiter_node..."
ros2 run doggobot arbiter_node >/tmp/arbiter_test.log 2>&1 &
ARB=$!
sleep 4
pgrep -f 'lib/doggobot/arbiter_node' >/dev/null || { echo "arbiter died on startup:"; cat /tmp/arbiter_test.log; exit 1; }

echo
echo "1. idle: no source publishing -> zero"
check "idle is zero" "0.000,0.000" "$(sample)"

echo
echo "2. behavior alone -> behavior drives"
B=$(pub_bg /behavior_cmd "{linear: {x: 0.20}, angular: {z: 0.10}}"); sleep 2
check "behavior passes through" "0.200,0.100" "$(sample)"

echo
echo "3. teleop outranks behavior"
T=$(pub_bg /teleop_cmd "{linear: {x: 0.30}, angular: {z: -0.20}}"); sleep 2
check "teleop wins" "0.300,-0.200" "$(sample)"

echo
echo "4. teleop goes stale -> falls back to behavior"
kill $T 2>/dev/null; sleep 2
check "fell back to behavior" "0.200,0.100" "$(sample)"

echo
echo "5. safety outranks everything except e-stop"
S=$(pub_bg /safety_cmd "{linear: {x: 0.0}, angular: {z: 0.0}}"); sleep 2
check "safety wins" "0.000,0.000" "$(sample)"
kill $S 2>/dev/null; sleep 2

echo
echo "6. output clamp: ask for full scale, expect the ceiling"
kill $B 2>/dev/null; sleep 1
B2=$(pub_bg /behavior_cmd "{linear: {x: 1.0}, angular: {z: 5.0}}"); sleep 2
check "throttle clamped to 0.150" "0.150," "$(sample)"
check "steering clamped to 0.8" ",0.800" "$(sample)"

echo
echo "7. e-stop overrides a live command"
ros2 topic pub -1 /estop std_msgs/msg/Bool "{data: true}" >/dev/null 2>&1; sleep 2
check "e-stop zeroes output" "0.000,0.000" "$(sample)"

echo
echo "8. e-stop clears and control returns"
ros2 topic pub -1 /estop std_msgs/msg/Bool "{data: false}" >/dev/null 2>&1; sleep 2
check "control restored after clear" "0.150,0.800" "$(sample)"

echo
echo "9. all sources stop -> back to zero"
kill $B2 2>/dev/null; sleep 2
check "returns to idle zero" "0.000,0.000" "$(sample)"

# `ros2 run` is a launcher whose child is the real node. Killing the launcher
# leaves the node alive, publishing zeros onto /cmd_vel forever. That orphan is
# what made the first bench steering test look like a wiring fault: the servo
# was being told 0.9 and 0.5 alternately, twenty times a second.
kill $ARB 2>/dev/null
pkill -f 'lib/doggobot/arbiter_node' 2>/dev/null
wait 2>/dev/null
sleep 1
if pgrep -f 'lib/doggobot/arbiter_node' >/dev/null 2>&1; then
  echo "WARNING: an arbiter survived cleanup:"; pgrep -af 'lib/doggobot/arbiter_node'
fi
echo
echo "================ $pass passed, $fail failed ================"
echo "arbiter log:"; sed -n '1,40p' /tmp/arbiter_test.log
exit $(( fail > 0 ))
