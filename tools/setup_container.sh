#!/usr/bin/env bash
# Re-apply Doggobot's container patches after the container has been recreated.
#
# WHY YOU WILL NEED THIS: Docker can only expose a device that existed at
# `docker run` time, so adding the pan-axis ESP32 to the Pi means recreating the
# container. A recreate throws away everything that was patched INSIDE it, and
# both of the things it throws away have already cost an evening once:
#
#   * /etc/resolv.conf, captured before Tailscale owned DNS on the host, so
#     tailnet names stop resolving and llm_node cannot reach the LLM host.
#   * ROS_DOMAIN_ID, because bashrc_docker.sh exports the class default 96 and
#     overrides whatever `docker run -e` set.
#
# This script does NOT create the container. Creating it is coursework and it is
# yours to do; run the class flow, then run this. What it prints first is the one
# flag that has to be added to that command.
#
#   ./tools/setup_container.sh            re-apply everything
#   ./tools/setup_container.sh --check    report only, change nothing
set -uo pipefail

NAME="${DOGGOBOT_CONTAINER:-Doggobot}"
DOMAIN_ID=66                 # class default is 96; ours is 66 to stay isolated
TAILNET="tail502ca5.ts.net"
CHECK=0
[ "${1:-}" = "--check" ] && CHECK=1

say()  { printf '%s\n' "$*"; }
ok()   { printf '  ok    %s\n' "$*"; }
bad()  { printf '  FIX   %s\n' "$*"; }

say "container: $NAME"

if ! docker inspect "$NAME" >/dev/null 2>&1; then
  cat <<EOF

Container "$NAME" does not exist yet. Create it with the class flow, adding the
pan axis to the device list:

    --device /dev/doggobot-pan

If the udev rule is not installed yet, use the raw node instead and fix it later:

    --device /dev/ttyUSB1

Install the rule on the HOST first so the name exists:
    sudo cp deploy/99-doggobot-serial.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger
EOF
  exit 1
fi

running=$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null)
[ "$running" = "true" ] || { say "container is not running: docker start $NAME"; exit 1; }

# 1. the pan device actually reached the container
say
say "pan axis device"
if docker exec "$NAME" test -e /dev/doggobot-pan 2>/dev/null; then
  ok "/dev/doggobot-pan present"
elif docker exec "$NAME" test -e /dev/ttyUSB1 2>/dev/null; then
  ok "/dev/ttyUSB1 present (set pan_node port accordingly, or install the udev rule)"
else
  bad "no pan device inside the container. It must be passed at 'docker run'"
  bad "time with --device; there is no way to add one to a running container."
fi

# 2. DNS
say
say "DNS"
if docker exec "$NAME" grep -q '100.100.100.100' /etc/resolv.conf 2>/dev/null; then
  ok "resolv.conf points at Tailscale"
else
  bad "resolv.conf predates Tailscale"
  if [ "$CHECK" = 0 ]; then
    docker exec "$NAME" bash -c "cat > /etc/resolv.conf <<EOF
nameserver 100.100.100.100
nameserver 1.1.1.1
search $TAILNET
EOF" && ok "rewritten"
  fi
fi

# 3. ROS_DOMAIN_ID
say
say "ROS_DOMAIN_ID"
current=$(docker exec "$NAME" bash -lc 'echo ${ROS_DOMAIN_ID:-unset}' 2>/dev/null | tr -d '\r')
if [ "$current" = "$DOMAIN_ID" ]; then
  ok "already $DOMAIN_ID"
else
  bad "login shell reports '$current', want $DOMAIN_ID"
  if [ "$CHECK" = 0 ]; then
    docker exec "$NAME" bash -c \
      "grep -q 'DOGGOBOT domain' /root/.bashrc || printf '\n# DOGGOBOT domain: must come AFTER bashrc_docker.sh, which exports 96\nexport ROS_DOMAIN_ID=%s\n' $DOMAIN_ID >> /root/.bashrc" \
      && ok "appended to /root/.bashrc"
  fi
fi

# 4. python deps
say
say "python packages"
for pkg in serial vosk requests; do
  if docker exec "$NAME" python3 -c "import $pkg" 2>/dev/null; then
    ok "$pkg"
  else
    bad "$pkg missing"
    if [ "$CHECK" = 0 ]; then
      name=$pkg; [ "$pkg" = serial ] && name=pyserial
      docker exec "$NAME" pip3 install --quiet "$name" && ok "installed $name"
    fi
  fi
done

say
say "done. Verify the pan axis end to end with:"
say "  docker exec -it $NAME bash -lc 'source /home/projects/ros2_ws/src/doggobot/tools/env.sh && ros2 run doggobot pan_node'"
