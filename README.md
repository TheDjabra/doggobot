# Doggobot

**Voice-Controlled Autonomous Navigation** — MAE/ECE 148 Team 4, UC San Diego, Summer Session 2
2026.

Cesar Montiel (ECE) · Hektoras Djabra (MAE) · Naveen Weedagama (MAE)

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

## Hardware

| Item | Notes |
|---|---|
| Raspberry Pi 5, 16 GB | RPi OS Bookworm, everything runs in a Docker container |
| VESC | `/dev/ttyACM0`, driven by the class `vesc_twist_node` |
| OAK-D Lite | detection, stereo depth and object tracking on the camera's own VPU |
| LD06 LiDAR | mounted highest for an unobstructed 360 view, `/dev/ttyUSB0` |
| Samson Go Mic | USB, cardioid, on-board speech |
| 4S pack | main power; camera and LiDAR fed via a DC-DC converter |
| Feetech STS3215 + bus adapter | camera pan axis (integration in progress) |

Wiring, measured drive characteristics and the traps we hit are in
[docs/hardware.md](docs/hardware.md).

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
