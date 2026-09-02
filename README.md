# Doggobot

**Voice-Controlled Autonomous Navigation**. MAE/ECE 148 Team 4, UC San Diego, Summer Session 2
2026.

Cesar Montiel (ECE) · Hektoras Djabra, Hrag Djabraian (MAE) · Naveen Weedagama (MAE)

---

## Abstract

A 1/10 scale autonomous car that takes spoken commands and carries them out, either as single
actions or as multi-step sequences with conditions attached. You can talk to it directly, or from
anywhere on the network through a phone web app that also gives you sticks, a kill switch, live
video, and telemetry.

It runs six ROS2 nodes of our own alongside the class `ucsd_robocar_hub2` framework, on a
Raspberry Pi 5 with an OAK-D Lite and an LD06 LiDAR. Perception runs on the camera's own
accelerator; speech runs either offline on the car or in the phone's browser; nothing that can
move the car does so until it is deliberately armed.

---

## What we promised, and what we delivered

### Must haves

| Promised | Delivered |
|---|---|
| Voice command input | **Yes.** Two independent paths: an on-board microphone running Vosk offline with a wake word, and browser speech recognition on a phone. Both publish to the same topic, so the car cannot behave differently depending on how it was told. |
| Command sequencing | **Yes.** Spoken chains: *"atlas forward then circle left then stop"*. Parsed by keyword split, no cloud service involved. |
| Autonomous driving | **Yes.** Person-following with a PD loop on bounding-box offset for steering and stereo depth for throttle. Validated on the ground. |
| Camera-based detection | **Yes.** A YOLO11n person detector we trained ourselves, running on the OAK-D Lite's Myriad X at 12.3 fps with stereo depth and on-camera tracking. Plus HSV colour detection for the mission cues. |
| Conditional actions | **Yes.** *"forward until green then circle left then stop"*. Steps wait on observed world state, with the step duration doubling as a timeout so an absent condition cannot strand the car. |

### Nice to haves

| Promised | Delivered |
|---|---|
| Object recognition beyond colour | **Yes**, the trained person detector, used for following. |
| Natural language / complex chains | **Partial.** Chains and conditions work by keyword parsing. An LLM tier for looser phrasing is designed but not built. |

### Beyond the proposal

- **Phone web app** as the control surface: push-to-talk, dual sticks, kill switch, arm gate,
  primitive buttons, live MJPEG video with the tracking overlay, and colour-threshold tuning with
  live sliders. Installable to a home screen and served over HTTPS.
- **LiDAR proximity guard** covering the ~290 degrees the camera cannot see.
- **Boots to phone control unattended**, verified from a cold power cycle with no terminal.

---

## How it works

Every behaviour publishes to its own topic and **exactly one node, the arbiter, publishes
`/cmd_vel`**. That single rule is the spine of the design: two publishers on a command topic is
not an error in ROS2, it is a race, and it presents as a hardware fault with clean logs. It bit us
twice during development and cost hours both times.

```
stt_node (mic, Vosk) ──┐
phone (browser STT) ───┼──→ /voice_cmd ──→ behavior_node ──→ /behavior_cmd ──┐
phone (buttons)     ───┘                        ↑                             │
                                          /follow_cmd                          │
phone (sticks) ───────────────→ /teleop_cmd     │                             ├──→ arbiter_node ──→ /cmd_vel ──→ VESC
                                                 │                             │
camera → perception_node → /target_state → follow_node                        │
                         └→ /condition_state                                   │
LiDAR → safety_node ────────────────────────────→ /safety_cmd ────────────────┘
phone (kill / arm) ─────────────────────────────→ /estop, /arm ───────────────┘
```

Arbiter priority, highest first: **arm gate → e-stop → safety → teleop → behaviour**. Every source
has a staleness timeout, so a crashed node or a phone that leaves wifi releases the car rather
than latching its last command.

Full detail in [docs/architecture.md](docs/architecture.md).

---

## Reproducing this

The goal is that you can clone this and rebuild the whole vehicle, not just read about it.
Everything needed is here or is fetched by a script; the only things you must supply are your
own hardware identifiers, listed at the bottom of this section.

### Run the phone app on its own, no robot required

The control surface has no build step and does not need the camera, the speech model or the
servo. On any machine with ROS2 Jazzy:

```bash
pip install fastapi uvicorn
colcon build --symlink-install --packages-select doggobot
source install/setup.bash
ros2 launch doggobot phone.launch.py
```

Open `http://<that machine>:8080`. You get the full interface: tabs, sticks, ARM and KILL,
telemetry panes and the guard control. Nothing moves, because nothing is publishing, and the
arbiter will report the sticks arriving. It is the fastest way to see how the thing works.

Browser speech is the one part that will not work over plain HTTP, because the microphone and
`SpeechRecognition` APIs do not exist outside a secure context. See
[docs/app-and-comms.md](docs/app-and-comms.md).

### The whole vehicle

```bash
pip install -r requirements.txt          # or: pip install -e '.[perception,speech]'
bash tools/get_vosk_model.sh             # speech model, 68 MB, not in git
# detector weights: see models/README.md, which explains what they are and how
# to rebuild them from the OpenVINO IR
sudo cp deploy/99-doggobot-serial.rules /etc/udev/rules.d/   # EDIT IT FIRST, see below
sudo udevadm control --reload && sudo udevadm trigger
colcon build --symlink-install --packages-select doggobot
ros2 launch doggobot all.launch.py
```

`docs/runbook.md` is the procedure end to end, including the container, publishing the page
over HTTPS, zeroing the pan axis and calibrating the drive.

`tools/install_service.sh` installs the systemd unit that brings the stack up at power-on, so
the vehicle boots to a working control page with no terminal involved.

### Check it without any hardware at all

Six suites run on a laptop with no ROS, no car and no camera, by stubbing the ROS client
library and driving the real nodes:

```bash
python3 tools/sim_cascade.py        # follow cascade against a vehicle model
python3 tools/test_search.py        # camera search sweep, on a fake clock
python3 tools/test_safety_guard.py  # LiDAR guard hysteresis and override
python3 tools/test_keywords.py      # command vocabulary and quantity parsing
```

### What you must change for your own setup

| Where | What, and why it cannot be shared |
|---|---|
| `deploy/99-doggobot-serial.rules` | USB serial numbers of **your** ESP32, LiDAR and VESC. Read them with `udevadm info -n /dev/ttyACM0`. Ours are in there as worked examples |
| `config/llm.yaml` | `host:` must point at your own Ollama machine. Optional: with it unreachable the offline keyword path still works |
| `docs/runbook.md`, `tools/install_service.sh` | Our Tailscale hostname. Substitute yours, or serve the page over plain HTTP on a LAN and accept losing browser speech |
| `config/follow.yaml` | `half_fov_deg` and `invert` depend on your camera crop and how the mount faces. `tools/measure_fov.py` measures the first in about 20 seconds |
| `config/behavior.yaml` | `metres_per_second`, `coast_metres`, `degrees_per_second`. Ours were measured with a tape measure; `tools/calibrate_motion.py` walks you through doing the same |
| `config/color_thresholds.json` | Lighting-dependent. The phone's TUNE tab has live sliders and a mask overlay |
| `web/index.html` | Branding is ours. The layout and logic are the reusable part |

Everything else should run as it stands.

---

## Hardware

Enough detail to order the parts, not just to recognise them. Items marked *course* came
with the MAE/ECE 148 kit and are listed for completeness.

### Vehicle

| Item | Detail that matters |
|---|---|
| 1/10 scale RC chassis | *course.* Ackermann steering, brushless motor |
| VESC | *course.* Enumerates at `/dev/ttyACM0`, driven by the class `vesc_twist_node` |
| Raspberry Pi 5, 16 GB | RPi OS Bookworm. Everything runs in a Docker container |
| **4S** LiPo | Main pack. **Set the VESC cutoff for 4S.** A 3S profile on a 4S pack puts the low-voltage cutoff at 2.25 V per cell, far below where a LiPo should ever be taken. Ours was wrong until 2026-08-29 |
| DC-DC converter | Steps the 4S pack down for the Pi, camera and LiDAR |

### Sensing

| Item | Detail that matters |
|---|---|
| Luxonis **OAK-D Lite** | Detection, stereo depth and tracking on the camera's own VPU. Draws enough that it wants the DC-DC rail, not the Pi's USB alone |
| **LD06** LiDAR | 360 scan, `/dev/ttyUSB0`. Mount it highest so the vehicle does not occlude it |
| **Samson Go Mic** | USB, class-compliant, **switch it to cardioid**. Omni picks up the Pi's own fan |

### Camera pan axis

| Item | Detail that matters |
|---|---|
| **Feetech STS3215** serial-bus servo | 7.4 V, 19 kg, 12-bit magnetic encoder. **Serial bus, not PWM**: the VESC's servo output is already committed to steering, and the loop needs the servo's *measured* angle, which a hobby PWM servo cannot report |
| **Waveshare Bus Servo Adapter (A)** | The servo driver. Half-duplex TTL UART to the servo bus. Jumper on **A (UART-SERVO)**. Servo power goes into this board's own screw terminal, never from the microcontroller |
| **ESP32-S3** dev board | Talks the servo bus and nothing else. Ours has **native USB**, which changes the firmware build, see [firmware/README.md](firmware/README.md) |
| **2S** LiPo, 2200 mAh | **Powers the servo, through the adapter.** Separate from the main pack on purpose: the servo is 7.4 V and the vehicle runs 4S, and a 19 kg servo's stall current is amps, not milliamps |
| 25T servo horn | Ships with the servo. Bolt pattern is 4 x M2 on a 14.0 mm circle |

Measured drive characteristics, the servo bring-up numbers and the traps we hit are in
[docs/hardware.md](docs/hardware.md). How the phone app is built, how it reaches the
vehicle and every message they exchange is in
[docs/app-and-comms.md](docs/app-and-comms.md).

### Software not in this repo

| | |
|---|---|
| `ucsd_robocar_hub2` | *course* framework, supplies `vesc_twist_node` and the `ldlidar` driver |
| Detector weights | Not in git. `models/README.md` says exactly what they are and how to rebuild them |
| Vosk speech model | `bash tools/get_vosk_model.sh` fetches it |
| Ollama host | Optional. Only the LLM slow path needs it, and it is a plain HTTP host in `config/llm.yaml` |

---

## Using it

Power on, open the app, tap **ARM**. Nothing responds until armed.

**To the on-board mic**, by name, since it listens continuously:

> "atlas forward" · "atlas circle left" · "atlas forward until green then stop"

**On the phone**, hold push-to-talk and drop the name, since holding the button is already the
gate:

> "circle left" · "forward then reverse" · "forward until green"

**Primitives**: forward, reverse, circle left, circle right, wait, stop, follow. All time-bounded.

Startup and troubleshooting: [docs/runbook.md](docs/runbook.md).

---

## Challenges, and what they taught us

Every one of these cost real time and is written up properly in
[build_log.md](build_log.md).

**A detector reporting 92% confidence can still be producing nonsense.** Our first working
pipeline emitted bounding boxes 24 times the image height at high confidence. The cause was one
wrong field in the model archive: the decoding `subtype` describes the Luxonis *export format*,
not the model's architecture generation. Confidence measures agreement with a decode, so a wrong
decode can be confidently wrong.

**Where a motor starts is not where it runs smoothly.** We measured the motor starting at 1000
ERPM and built the whole throttle law on it. At that speed the FOC speed controller could not hold
a setpoint at all: commanded 993 ERPM it wandered over a 1676 ERPM range while drawing 14.6 A,
against 3.0 A at cruise. The usable floor is more than double the starting threshold, and the
class calibration had been saying so all along.

**A safety system must observe intent, not an outcome it is already influencing.** The LiDAR guard
first read `/cmd_vel` to decide which way the car was going, but that is the arbiter output the
guard itself vetoes: stopping the car made it conclude nothing was moving, release, and let the car
lurch. It now reads the arbiter's inputs.

**A partially understood command must be refused, not partially executed.** "Forward then jump then
stop" originally drove the car forward, and "forward until purple" ran a plain forward. Silently
doing a fragment of what someone asked is worse than doing nothing, because they have no way to
know which fragment they got.

**Confirm the experiment ran before interpreting the result.** We spent several rounds diagnosing a
microphone that turned out to be recording an empty room.

---

## Layout

```
doggobot/     ROS2 python package: the six nodes
launch/       launch files (all.launch.py is the full stack)
config/       YAML parameters and colour thresholds
tools/        test harnesses and hardware probes
docs/         architecture, hardware, runbook
models/       model metadata and rebuild instructions
build_log.md  dated engineering record
```

## License

MIT.
