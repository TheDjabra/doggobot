# Build log

Dated engineering record: what was done, what broke, and why decisions went the way they
did. Written as the work happens, not reconstructed afterward. Source material for the
final report and the project writeup.

---

## 2026-08-25 - Container and repository set up

**Container.** The final project runs in its own container, `Doggobot`, cloned from the
graded-assignment container `robocar_team_4` rather than built fresh from the upstream
image. The reason is that three fixes made during the assignment live only in that
container's filesystem and would be lost by pulling the stock image again:

1. A settle delay added to `pyvesc/VESC/VESC.py`, fixing a partial-read race where the
   library decoded a 43-byte VESC reply after only 4 bytes had arrived and returned the
   string `None`.
2. `setuptools` pinned to 69.5.1, because the shipped 82.0.1 is above `colcon-core`'s own
   declared ceiling and breaks `--symlink-install`.
3. A corrected `source_ros2` alias that also sources `sensor2_ws`, without which the OAK-D
   packages are invisible to ROS2.

Cloned with `docker commit robocar_team_4 team4_final:base`, then run as `Doggobot`.
Verified after creation that all three fixes survived.

**Domain ID.** Set to 66 to keep this node graph separate from the assignment container's.
This had to be changed in two places, not one: the image's own `bashrc_docker.sh` exports
`ROS_DOMAIN_ID=96` on every interactive shell and overrides whatever `docker run -e` set,
so the authoritative value is the later export in `/root/.bashrc`. Verified through an
actual interactive shell rather than trusting the flag.

**Flags changed from the class `robocar_docker()` function.** Dropped `--device /dev/video0`,
which does not exist on this Pi (only `/dev/video19` through `23`, the ISP and codec nodes)
and would have made `docker run` fail. `--privileged` shares the host `/dev` regardless,
which is how the VESC and OAK-D are reachable.

**Added a source bind mount**, `/home/pi/doggobot` on the host to
`/home/projects/ros2_ws/src/doggobot` in the container. The class container mounts no
source volume, so any code written inside it exists only in a container layer, is not in
version control, and dies with the container or at kit return. A bind mount can only be
added at container creation, so it was free now and would have cost a full recreate later.

**Inherited calibration.** The clone carries the assignment's tuned values, so this
container can drive the ROS2 laps as-is: `Kp 0.2 / Ki 0.0 / Kd 0.1`, steering limits
+/-0.8, `max_throttle 0.382`, `min_throttle 0.363`.

**Note:** the two containers must not run at the same time. Separate domain IDs keep their
node graphs apart but do not stop them contending for the same VESC serial port and the
same OAK-D on the USB bus.

**Repository.** Scaffolded as an `ament_python` package so `colcon build` finds it, and
pushed to GitHub. The Pi holds a working clone at the bind-mount path.

---

## 2026-08-25 (lab) - Tailscale, arbiter_node, first bench motion

**Tailscale on the car.** Installed and joined the tailnet as `doggobot`
(`100.108.50.81`, `doggobot.tail502ca5.ts.net`). Two reasons, one immediate and one
structural. Immediate: campus wifi uses client isolation, so mDNS stopped resolving
`ucsdrobocar-148-04.local` the moment we left the home network, and the car became
unreachable. Structural: `tailscale cert` issued a real publicly-trusted certificate for that
hostname, which is what the phone app needs. The Web Speech API refuses to hand a page a
microphone on an insecure origin, so this was not convenience, it was a prerequisite for
voice input. Verified rather than assumed by issuing a cert and then deleting it.

Note: on campus wifi the tailnet connection routes through a relay in LA rather than
peer-to-peer, which adds latency. That should resolve when the phone and car are on the same
hotspot at demo time, but it needs measuring before we stream joystick input at 20-30 Hz.

**`arbiter_node`.** Written and passing a 10-case logic test with no actuator attached:
priority order, per-source staleness timeouts, latched e-stop, and output clamping. The test
harness lives at `tools/test_arbiter.sh` and is repeatable.

**`tools/env.sh`.** The first test run failed with `Package 'doggobot' not found`. Cause:
`source_ros2` is a shell *function* defined in the image's `/home/scripts/bashrc_docker.sh`,
and functions and aliases do not exist inside a child script. Sourcing that file directly is
not a fix either, because it ends with `export ROS_DOMAIN_ID=96` and would have silently moved
the project onto the wrong domain. So the environment is now explicit and version-controlled:
domain 66, every driver workspace, then ours last.

**Throttle units, corrected before commanding motion.** `vesc_twist_node` reads the
calibration file and computes `max_rpm = max_throttle * max_rpm = 0.382 * 20000 = 7640` ERPM,
then commands `rpm = 7640 * msg.linear.x`. So Twist `linear.x` is a *fraction* of an
already-capped range, not the calibration value itself. The arbiter's `max_throttle` is
therefore a second, project-level speed limit, and it was initially set to 0.382 under the
wrong assumption. Lowered to 0.15 (roughly 1150 ERPM) as a deliberately slow bench default, to
be raised only after speed is measured on the ground.

Also worth noting: `vesc_twist_node`'s callback does **not** clamp steering to
`max_left_steering`/`max_right_steering`. It computes those limits and never applies them. The
arbiter's clamp is therefore the only thing preventing an out-of-range servo command.

**First bench motion, and the bug it caught.** With the car on blocks, a steering sweep
produced servo noise but no visible wheel movement. `ros2 topic info /cmd_vel` reported
**Publisher count: 2**. An orphaned `arbiter_node` from the earlier test run was still alive
and publishing zeros at 20 Hz, so the VESC was receiving 0.8 and 0.0 alternately and the servo
was being commanded to 0.9 and back to center twenty times a second.

Root cause: `ros2 run` is a launcher and the node is its child, so `kill $!` in the test script
killed the launcher and orphaned the node. Fixed in `tools/test_arbiter.sh` by killing the node
by name and by adding a preflight check that refuses to run when an arbiter is already alive.

This is worth remembering as a class of bug rather than an incident: **the symptom looked
exactly like a hardware fault** (servo buzzing, nothing moving), and nothing in either node's
logs was wrong. `ros2 topic info` was the tool that found it in seconds. It is also a direct
vindication of the single-publisher rule the whole architecture is built on.

**Silent motor, and the measurement that explained it.** First throttle commands (0.05, 0.10)
produced no wheel movement. Rather than guess, `tools/vesc_probe.py` was written to talk to the
VESC directly with ROS out of the picture: read telemetry, then step RPM and read back what the
motor actually did. Result: firmware 6.6.55, 15.1 V input, no fault codes, and a motor start
threshold **between 500 and 1000 ERPM**. The failing commands were 382 and 764 ERPM. Not a
fault, a deadband.

That also exposed a mistake made earlier the same session. The arbiter's throttle ceiling had
been lowered to 0.15 on the reasoning that `vesc_twist_node` already caps at 0.382. It does, but
the class's own `lane_guidance_node` publishes `linear.x` between 0.363 and 0.382, so 0.15 was
*below the speed the car is calibrated to drive at*. Ceiling restored to 0.382, with the
measured floor of 0.13 recorded as a parameter and published in the arbiter's status so
behaviours can respect it.

**A second two-publisher incident, same root cause, different mechanism.** A cleanup command
`pkill -f arbiter_node` matched the very shell that was running it, so the shell killed itself
partway through and the remaining kills never executed. A subsequent launch then created a
second stack. Fixed with `tools/stop_stack.sh`, which matches installed binary paths using
bracket patterns (`arbiter_nod[e]`) that cannot match its own command line, and verifies nothing
survived. The lesson generalises: `ros2 topic info /cmd_vel` reporting a publisher count other
than 1 is now the first thing checked before any bench test, and both test harnesses refuse to
run otherwise.

**Battery.** Measured 15.1 V, so a **4S** pack, which is what every team received. The recorded
VESC motor-wizard configuration says 3S Li-ion. If that is still set, the low-voltage cutoff
will not protect this pack. Flagged for checking in VESC Tool.

**Bench results, all with the car on a stand and exactly one publisher on /cmd_vel:**

- Steering: full right, center, full left, center. Polarity correct, right command gives right.
- Throttle: 0.20 and 0.37 both spin at visibly different speeds. Reverse works at -0.25 and
  -0.37, no brake-then-reverse sequence needed.
- **Deadman**: teleop publisher killed mid-drive with no stop command sent, car stopped itself.
- **E-stop**: engaged while a 0.25 throttle command was still streaming, output zeroed; cleared,
  control resumed with nothing republished.

The drive path is therefore proven end to end: `ros2 topic pub` -> arbiter -> `/cmd_vel` ->
`vesc_twist_node` -> VESC -> wheels, with both safety paths validated on real hardware before
any autonomous behaviour exists.

---

## 2026-08-25 (lab, later) - Phone control surface

**Tailscale on the car.** Joined the tailnet as `doggobot`
(`doggobot.tail502ca5.ts.net`). Two reasons. Campus wifi uses client isolation, so mDNS stopped
resolving the car entirely the moment we left the home network. And `tailscale cert` issues a
real publicly-trusted certificate for that hostname, which is a hard prerequisite for the voice
work: the Web Speech API refuses to give a page a microphone on an insecure origin. Verified by
issuing a cert, not assumed.

**`voice_bridge_node`.** FastAPI, a WebSocket, and `rclpy` in a single process. rclpy spins in a
background thread and the socket handler publishes into it.

    button / stick  ->  /teleop_cmd   (Twist)
    kill switch     ->  /estop        (Bool)
    speech          ->  /voice_cmd    (String JSON)   [transport live, recognition not attached]

One process rather than rosbridge because a server has to exist anyway to serve the page over
HTTPS and later to proxy the LLM call so the key never reaches the browser. Given that,
rosbridge would add a second server and port for nothing.

**Design decisions in the page itself**, each for a reason worth keeping:

- **Teleop is a stream, not a command.** The page repeats the current stick position 20x a
  second. It never sends "drive until told otherwise". A phone that sleeps, locks, or leaves
  wifi simply stops transmitting and the arbiter releases the car within 0.5 s. This is the
  deadman already validated on hardware, reached from the phone side.
- **The kill switch is one WebSocket frame straight to `/estop`.** It depends on no model, no
  recognition, and no network hop beyond the bridge.
- **Sticks spring back to centre on release**, and each pad uses pointer capture so dragging a
  thumb off the pad still tracks rather than sticking at its last value. A car stick that held
  its position would be a runaway waiting to happen.
- **Speed slider starts at 0.13**, the measured motor floor, and the bridge steps smaller
  requests up to it, so a low setting still moves the car instead of feeling like a dead button.
- **Last client out stops teleop**, borrowed from the turret app's rule of disarming the payload
  when the last viewer disconnects.
- **Two-stick RC transmitter layout**: left vertical is throttle, right horizontal is steering,
  so muscle memory transfers. Each pad captures its own pointer, so both thumbs work at once.
  Landscape puts them at the screen edges with the controls between.
- **Push-to-talk, not continuous listening**, even in the skeleton. Continuous recognition on
  Android Chrome duplicates results and restarts unpredictably, and a held button doubles as a
  safety gate against the car acting on overheard speech.

Styling reuses the AI TURRET HUD's design tokens directly (square corners, bone white on
near-black, amber accent, condensed display over mono data) so the two projects read as one
system. Written as a single self-contained page rather than a React build, deliberately: a Vite
toolchain would have consumed the whole available window for no functional gain yet.

**Bug found and fixed:** `self.clients` collides with `rclpy.node.Node.clients`, a read-only
property listing a node's service clients. The node died at construction with
`property 'clients' has no setter`. Renamed to `client_count`.

**Measured**: 35 ms round trip from phone to car and back, over the Tailscale relay, on campus
wifi. That is comfortably inside what a 20 Hz stick stream needs, and much better than feared.
Worth re-measuring at Warren Mall on the phone hotspot, where it should improve further because
Tailscale can then connect peer-to-peer over the local network instead of relaying.

---

## 2026-08-26 (lab) - Perception: getting the detector running under DepthAI v3

**GPS removed**, LiDAR fitted. The USB bus is now VESC on `ttyACM0`, LD06 on `ttyUSB0`, OAK-D
on USB, which is tidier than before and frees power budget.

**The container runs depthai 3.1.0, not 2.x**, and the class's own `multi_cam_node.py` is
written against the v3 API (`pipeline.create(dai.node.Camera).build(...)`,
`requestOutput(...).createOutputQueue()`). Prior research notes said to pin v2; doing so would
have broken the course camera node. v3 has everything the follow design needs: `NNArchive`,
`DetectionNetwork`, `SpatialDetectionNetwork`, `ObjectTracker`, `StereoDepth`.

**v3 configures detection through an NN Archive, not setters.** `DetectionNetwork` has no
`setNumClasses`. Rather than re-export the model through a browser, `tools/make_nn_archive.py`
reads the blob's own tensor names and shapes via `dai.OpenVINO.Blob()` and generates a
schema-valid archive around the existing file.

**The subtype trap.** The head metadata's `subtype` must match the export format, not the
architecture generation. Measured against a single person in frame:

- `yolov6`: confidence 0.92, **garbage boxes** (normalised sizes like 8.1 x 24.0, centres
  outside the frame), ~10 per frame
- `yolov8`: one sane box, confidence only ~0.55 (reads 5 channels from a 6-channel tensor)
- `yolov6r2`: one box, confidence 0.94, stable. **Correct.**

The important lesson is that **the wrong subtype reported high confidence**. `yolov6` looked
like a working detector by every metric except the coordinates themselves, and a check of the
form "is it detecting anything" would have passed it while the follow loop chased boxes many
times the size of the image. Normalised box sizes must be within 0..1; printing them is what
found this in one run.

**Validated**: 23.0 fps at 416 (vs 25.6 natively under v2, so ~10% for the v3 pipeline and
letterboxing), 1.07 detections per frame with one person present, best confidence 0.95.

New tools: `tools/make_nn_archive.py`, `tools/probe_blob.py`, `tools/probe_detections.py`.

### Perception pipeline: measured costs, and the tracker trap

Built `tools/probe_perception.py` to measure the full pipeline (stereo depth +
spatial detection + tracking) before writing any ROS2 node that depends on it. Three
resource walls, each of which would have been a mystery later.

**1. SHAVE allocation.** The 6-shave blob refused to start: `Blob compiled for 6 shaves, but
only 5 are available`. With StereoDepth, ImageManip, SpatialLocationCalculator and ObjectTracker
each claiming cores, only 5 remain for the network. Recompiled the OpenVINO IR with
`blobconverter(shaves=5)`; no re-export from the original weights needed.

**2. Stereo median filter out of SIPP memory.** `'Median' out of system resources: '126'`.
Turned the median filter off (`initialConfig.postProcessing.median = MEDIAN_OFF`). Depth is
slightly noisier, which the host-side median we apply to Z compensates for anyway.

**3. The ObjectTracker is the bottleneck, and `setRunOnHost(True)` is the fix.**

| Configuration | fps |
|---|---|
| detector alone | 23.0 |
| + stereo depth | 7.6 |
| + tracker on-device (either type) | **1.2** |
| + tracker on host | **11.3** |
| + host tracker, FAST_DENSITY stereo | **12.1** |

Once the NN and stereo have taken their SHAVEs the on-device tracker is allocated exactly one
core and starves. Changing tracker type does not help (colour histogram 0.9, imageless 1.2); the
allocation is the problem, not the algorithm. The Pi has four idle Cortex-A76 cores, so running
it host-side costs essentially nothing and returns a 10x speedup.

**Validated at 12.1 fps**: stable tracklet id, TRACKED on 116 of 121 frames with LOST appearing
briefly as expected, depth following a person between 544 and 1034 mm.

**Model choice, settled on data.** The custom 416 single-class detector gives 12 fps end to end.
The stock COCO export is 640 with 80 classes, roughly 2.4x the pixels and a far wider head,
which would land near 5 fps in the same pipeline. If more classes are ever wanted, train a small
multi-class model at 416 rather than adopt the stock 640 one.

### Power stress test under motor load (2026-08-26)

The OAK-D and LiDAR run from a DC-DC converter off the same 4S pack that feeds the VESC, so
motor current transients sag the source that powers the camera. That failure cannot appear in a
static test, only under motion, which is how it would otherwise be discovered at a demo.
`tools/stress_power.sh` runs perception for 60 s while commanding eight full accelerate and
direction-reversal cycles.

| Measure | Result |
|---|---|
| perception fps under load | **12.3** (12.1 idle: no degradation) |
| tracker | TRACKED 731/738 frames, LOST twice |
| depth | 426 to 2054 mm |
| `vcgencmd get_throttled` | **0x0** |
| FET / SoC temp | 53.2 C |
| USB resets during the run | none |

`0x0` matters more than it looks: bit 16 of that register **latches** if undervoltage has
occurred at any point since boot, so a clean zero after eight hard cycles means the pack never
sagged enough to disturb the Pi and the converter held 5 V through every transient.

The two USB disconnects in `dmesg` are the OAK-D's normal boot and shutdown transitions, where
it re-enumerates between `Movidius MyriadX` (unbooted, `03e7:2485`) and `Luxonis Device`
(running, `03e7:f63b`). Not brownouts.

**Caveat**: the LiDAR was connected but its driver was not streaming, so this measures camera
plus motor, not full peripheral load. Repeat once the LiDAR node runs.

This demotes the powered USB hub from necessary to headroom.

### Full peripheral load: LiDAR + camera + motor (2026-08-26)

The LD06 runs from the class framework's own driver, not the full `all_nodes.launch.py` the
LiDAR assignment used (that launch also starts `lane_guidance_node`, which publishes `/cmd_vel`
and would collide with the arbiter). Just the driver:

    ros2 launch ldlidar ldlidar.launch.py serial_port:=/dev/ttyUSB0

`tools/env.sh` already sources that workspace. Publishes `/scan` at exactly 10 Hz.

Repeated the power stress test with the LiDAR streaming as well:

| Measure | Camera only | Camera + LiDAR |
|---|---|---|
| perception fps | 12.3 | **12.3** |
| `/scan` | n/a | **10.000 Hz held** |
| `get_throttled` | 0x0 | **0x0** |
| temperature | 53.2 C | 53.8 C |
| USB resets | none | **none** |

The LiDAR (direct on a Pi USB 2.0 port, ~2 KB/s) costs nothing measurable. **The powered USB hub
is optional headroom, not a requirement.** Tested in the current wiring deliberately: testing the
hub first would have proven nothing about whether it was needed, and the un-hubbed configuration
is also what exists if the hub ever fails in the field.

**Lock-on state machine validated under load.** This run's tracker statuses:
`NEW 9, TRACKED 666, LOST 187, REMOVED 2`, three distinct tracklet ids. The target was lost and
re-acquired repeatedly out to 2474 mm, so the design behaved as intended: TRACKED while visible,
LOST as a grace period, REMOVED on real departure, and a **new id on re-entry** rather than a
silently reused one, which is the `UNIQUE_ID` policy making "have I lost my target"
unambiguous. REMOVED is the signal the pan-servo sweep will trigger on.
