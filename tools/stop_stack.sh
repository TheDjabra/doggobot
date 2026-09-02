#!/usr/bin/env bash
# Stop every Doggobot ROS process cleanly, and verify none survived.
#
# Run INSIDE the container:
#   docker exec Doggobot bash /home/projects/ros2_ws/src/doggobot/tools/stop_stack.sh
#
# Why this is a script and not `pkill -f arbiter_node`: that pattern matches the
# shell running it, so the shell kills itself partway through the command and the
# rest never executes. Everything below uses bracket patterns (arbiter_nod[e])
# which cannot match the literal text of this file's own command line, and
# matches the INSTALLED binary path rather than a bare name.

kill_pat() {
  local pat="$1" name="$2"
  if pgrep -f "$pat" >/dev/null 2>&1; then
    echo "  stopping $name"
    pkill -f "$pat"
    sleep 1
    pgrep -f "$pat" >/dev/null 2>&1 && { echo "  force-killing $name"; pkill -9 -f "$pat"; sleep 1; }
  fi
}

kill_pat 'ros2 launc[h] doggobot'                  'ros2 launch'
kill_pat 'lib/doggobot/arbiter_nod[e]'             'arbiter_node'
kill_pat 'lib/doggobot/voice_bridge_nod[e]'        'voice_bridge_node'
kill_pat 'lib/doggobot/perception_nod[e]'          'perception_node'
kill_pat 'lib/doggobot/follow_nod[e]'              'follow_node'
kill_pat 'lib/doggobot/behavior_nod[e]'            'behavior_node'
kill_pat 'lib/doggobot/stt_nod[e]'                 'stt_node'
kill_pat 'lib/doggobot/safety_nod[e]'              'safety_node'
kill_pat 'lib/doggobot/llm_nod[e]'                 'llm_node'
# pan_node holds a serial port EXCLUSIVELY in practice, so an orphan of it does
# not just linger, it locks the new instance out: the replacement loops on
# "device reports readiness to read but returned no data (multiple access on
# port?)" and the camera never moves. Observed 2026-09-01, when a pan_node from
# an earlier launch survived a service restart because this list predated it.
kill_pat 'lib/doggobot/pan_nod[e]'                 'pan_node'
kill_pat 'ldlidar/ldlida[r]'                       'ldlidar'
kill_pat 'ucsd_robocar_actuator2_pkg/vesc_twist_nod[e]' 'vesc_twist_node'
kill_pat 'topic pu[b]'                             'stray topic pub'

sleep 1
left=$(pgrep -af 'lib/doggobot/arbiter_nod[e]|lib/doggobot/voice_bridge_nod[e]|lib/doggobot/pan_nod[e]|vesc_twist_nod[e]|ros2 launc[h] doggobot' 2>/dev/null)
if [ -n "$left" ]; then
  echo "STILL RUNNING:"; echo "$left"; exit 1
fi
echo "stack stopped, nothing left"
