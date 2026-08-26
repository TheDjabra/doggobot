# Runbook: from a laptop to driving from your phone

Every command here runs on the Pi unless marked otherwise.

## 0. First time only, on the phone

Install Tailscale, log into the same account, and confirm the car appears as `doggobot`.

## 1. Connect

```bash
ssh doggobot            # works from any network, over the tailnet
ssh robocar             # only on the same LAN, uses mDNS
```

`doggobot` is the reliable one. Campus wifi uses client isolation, which breaks mDNS, so
`robocar` will fail there.

## 2. Start the container

```bash
docker start Doggobot
```

Containers do not auto-start after a reboot. Nothing else is needed; the code lives on a bind
mount at `/home/pi/doggobot` and is already visible inside.

## 3. Stop anything left running

```bash
docker exec Doggobot bash /home/projects/ros2_ws/src/doggobot/tools/stop_stack.sh
```

**Do not skip this.** A surviving node from a previous session leaves two publishers on
`/cmd_vel` or holds port 8080, and both failures look like hardware problems rather than
software ones. The script reports what it stopped and verifies nothing survived.

## 4. Pull and build (only after code changes)

```bash
cd /home/pi/doggobot && git pull
docker exec Doggobot bash -c "cd /home/projects/ros2_ws && colcon build --symlink-install --packages-select doggobot"
```

## 5. Launch

```bash
docker exec -d Doggobot bash -c \
  "source /home/projects/ros2_ws/src/doggobot/tools/env.sh && ros2 launch doggobot phone.launch.py > /tmp/phone.log 2>&1"
```

Starts `vesc_twist_node`, `arbiter_node`, and `voice_bridge_node`. Use `docker exec -it` without
`-d` instead if you want the logs in front of you.

## 6. Sanity check before touching the car

```bash
docker exec Doggobot bash -ic "source_ros2 && ros2 topic info /cmd_vel"
```

**Publisher count must be exactly 1.** Anything else means step 3 did not fully take.

## 7. Publish the page over HTTPS

```bash
sudo tailscale serve --bg 8080
```

Persists across reboots, so this is usually already done. Check with `tailscale serve status`.
HTTPS is not cosmetic: the Web Speech API refuses to give a page a microphone on an insecure
origin, so the voice tab can never work over plain `http://<ip>`.

## 8. On the phone

Open **https://doggobot.tail502ca5.ts.net**, then Chrome menu → *Add to Home screen* to install
it as a standalone app. Rotate to landscape for the gamepad layout: left stick throttle, right
stick steering.

## Shutting down

```bash
docker exec Doggobot bash /home/projects/ros2_ws/src/doggobot/tools/stop_stack.sh
docker stop Doggobot
sudo shutdown -h now
```

Then wait for the green activity LED to stop flashing before disconnecting power. The red LED
stays lit; that is the Pi 5's standby state, not a running system.

## When something is wrong

| Symptom | First thing to check |
|---|---|
| Servo buzzes, nothing moves | `ros2 topic info /cmd_vel` publisher count |
| Wheels do not turn at all | Throttle below the 0.13 motor floor. See `docs/hardware.md` |
| Page will not load | `tailscale serve status`, then `curl localhost:8080/healthz` on the Pi |
| Page loads, no microphone | Not served over HTTPS, or the tab is not push-to-talk |
| Device missing inside container | `docker restart Doggobot`, then re-copy the X11 cookie |
