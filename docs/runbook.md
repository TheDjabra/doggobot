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

### Zeroing a new mount

**Zero the servo, do not carry an offset in software.** The horn fits in 14.4 degree
steps, so mechanical straight-ahead never lands on the encoder's centre, and the encoder
covers exactly one turn. On this mount the horn came out 128 degrees off, which left
only about 52 degrees of travel on one side before the count passed 4095 and **wrapped**
instead of clamping. A software offset cannot fix that; it just moves where it breaks.

The firmware boots limp for this, so the sequence is safe:

```bash
# 1. axis is free. Fit the horn and point the camera down the centreline.
~/esptool-venv/bin/python3 tools/pan_console.py --port /dev/ttyACM1 --watch 60

# 2. store this position as zero, in the servo's EEPROM. Survives power cycles.
~/esptool-venv/bin/python3 tools/pan_console.py --port /dev/ttyACM1 z

# 3. walk out to the limits, aborting on binding or high load
~/esptool-venv/bin/python3 tools/pan_console.py --port /dev/ttyACM1 --limits
```

Done on the car 2026-09-01: 128.06 to 0.00 degrees, then every angle out to +/-90
reached with worst error 1.9 degrees and zero bus errors.

### Calibrating the mount

`invert` depends on the bracket, and `half_fov_deg` must be measured. Both live in
`config/pan.yaml` and `config/follow.yaml`:

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

## Overriding the LiDAR guard for bench and stand work

On a stand the wheels are off the ground and the guard is measuring the bench, a chair
leg or your own foot. Those are real obstacles and it is right to stop for them, but
they are not relevant to what you are testing, and the result is that every command is
vetoed a moment after it starts.

The phone has a **Lidar Guard** button below KILL. Tap it to override.

- It defaults to **on**, and comes back on after a restart. That is deliberate.
- It **re-enables itself** when the last client disconnects, at the same time as the
  disarm. An override is something you hold for a specific test, not something you set.
- While overridden the button is amber and pulsing, and it keeps reporting what the
  guard can still see, for example `would stop: right`. The guard has not stopped
  working; only its veto is withheld.

If the guard is tripping when nothing is actually near the car, do not reach for the
override first. Check what it is seeing:

```bash
docker exec Doggobot bash -c "source /home/projects/ros2_ws/src/doggobot/tools/env.sh \
  && python3 /home/projects/ros2_ws/src/doggobot/tools/lidar_sectors.py 15"
```

A return at a **fixed bearing and a very short range** is the car seeing part of itself,
usually a cable or a bracket that has moved into the scan plane. Find it and move it.
Only once you know what is inside the envelope should you raise `min_range_m` in
`config/safety.yaml` from 0.02 to about 0.10, and that number is the difference between
ignoring your own wiring and ignoring somebody's foot.

Checking the guard's logic without a car or a LiDAR:

```bash
python3 tools/test_safety_guard.py
```
