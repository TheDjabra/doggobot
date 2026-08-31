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
| `Name or service not known` for a tailnet name | the container's `/etc/resolv.conf` predates Tailscale owning DNS on the host. See below |

## Container DNS, if tailnet names fail inside it

Symptom: `rasputin` resolves on the Pi but not inside the container, so `llm_node` reports
`Name or service not known` while `curl` from the host works.

Cause: Docker captured `/etc/resolv.conf` when the container was **created**, which was before
Tailscale took over DNS on the Pi. The container therefore lists the ISP's resolvers and knows
nothing about MagicDNS.

Docker does not regenerate that file once it has been edited, so the fix persists across
restarts, but **it is lost if the container is recreated**:

```bash
docker exec Doggobot bash -c 'cat > /etc/resolv.conf <<EOF
nameserver 100.100.100.100
search tail502ca5.ts.net
nameserver 68.105.28.11
nameserver 68.105.29.11
EOF'
```

Verify with `docker exec Doggobot getent hosts rasputin`.

## Camera pan axis

### Bench check, no ROS and no car

The firmware talks a line protocol, so the axis can be exercised from any machine it is
plugged into:

```bash
cd ~/projects/doggobot/firmware
arduino-cli compile --fqbn esp32:esp32:esp32s3 -u -p /dev/ttyUSB0 esp32_pan
cd .. && python3 tools/pan_console.py --selftest
```

The self-test commands 0, +/-30 and +/-60 degrees and reports **measured against commanded**
for each, plus bus errors and supply voltage. A pass means the servo went where it was told,
which is a different and much more useful claim than the servo moved.

If the scan finds nothing, work the list in this order, because it is ordered by how often
each one is actually the cause:

1. TX and RX crossed. This was the fault the first time. Adapter TX to GPIO 17, adapter RX to
   GPIO 16.
2. Jumper not on **A (UART-SERVO)**.
3. Servo not powered, or the adapter's own power input empty. The servo does not draw power
   from the signal wires.
4. No common ground between the adapter and the ESP32.

### Putting it on the Pi

Install the udev rule on the **host** first, so a stable name exists before anything asks for
it. The Pi already has the VESC and the LiDAR on `/dev/ttyUSB*` and kernel numbering follows
probe order, which is not a promise:

```bash
sudo cp deploy/99-doggobot-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
ls -l /dev/doggobot-pan
```

**A container cannot be given a device it was not started with.** Adding the ESP32 means
recreating the container, and a recreate discards everything patched inside it. Create it the
usual way, with the pan device added to the device list:

```
--device /dev/doggobot-pan
```

Then re-apply this project's patches, which is what would otherwise be rediscovered at the
worst possible moment:

```bash
./tools/setup_container.sh          # or --check to report without changing
```

That restores `/etc/resolv.conf` for tailnet DNS, re-appends `ROS_DOMAIN_ID=66` after the
class `bashrc_docker.sh` has exported 96, and installs `pyserial`.

### Calibrating the mount

Two numbers depend on the bracket and cannot be guessed. Both live in `config/pan.yaml`:

```bash
ros2 launch doggobot pan.launch.py
ros2 topic pub -1 /pan_cmd std_msgs/msg/Float32 '{data: 30.0}'
```

- `invert` : if `+30` swings the camera left, set it true.
- `centre_offset_deg` : trim so `0` looks straight down the chassis centreline.

Then measure `half_fov_deg` in `config/follow.yaml`. It is **not** the camera's spec-sheet
HFOV, because `x` is normalised across the detector's letterboxed input. Stand at a known
bearing, read `x` from `/target_state`, and divide the angle by `x`. Getting this wrong scales
the whole outer loop.

Finally measure `capture_lag_s`: the age of a frame when `follow_node` sees it. At a 120 deg/s
slew every 100 ms of it is 12 degrees of error injected into the bearing.

### Checking the loop without a car

```bash
python3 tools/sim_cascade.py            # 7 gated checks, no ROS required
python3 tools/sim_cascade.py --plot     # ascii trace of bearing over time
```

Two of those checks deliberately invert a sign and require the run to diverge, so the suite is
known to be capable of seeing the failure it rules out.
