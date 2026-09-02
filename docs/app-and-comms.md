# The app, and how it talks to the robot

Everything an operator touches is one web page. This is how it is built, how it reaches the
vehicle, and what every message on the wire means.

## Why a web app at all

A phone is the only controller that is always to hand, needs no pairing, and already has a
microphone, a screen and a browser. Making the control surface a web page means there is
nothing to install on it, nothing to keep in step with the robot, and no second codebase.

## How it is built: it is not

`web/index.html` is a **single file with no build step**. No framework, no bundler, no
`node_modules`, no npm. Open it in an editor, change it, reload the page. The only external
dependency is the browser's own APIs.

That is a deliberate choice rather than a shortcut. A build step is a thing that breaks on
somebody else's machine at the worst moment, and this page is the emergency stop for a moving
vehicle. Fewer moving parts between the source and the screen is worth more here than any
convenience a framework buys.

| File | What it is |
|---|---|
| `web/index.html` | The whole application: markup, CSS, and the client script |
| `web/manifest.webmanifest` | Makes it installable, so it opens without browser chrome |
| `web/icon-*.png`, `favicon.svg` | Home-screen icons |

Installing it to the home screen matters more than it sounds: with the address bar and tabs
gone, the KILL button cannot be missed by a thumb aiming for it in a hurry.

## How it is served

`voice_bridge_node` is one process running **two things at once**: a FastAPI server on port
8080, and a ROS2 node. `rclpy` spins in a background thread while `uvicorn` owns the main
loop. That is why there is no separate web backend to deploy: the thing serving the page is
also the thing publishing to `/cmd_vel`'s upstream topics, so a message from the phone
becomes a ROS message inside the same process, with no broker or bridge in between.

| Route | Purpose |
|---|---|
| `GET /` | The page |
| `GET /ws` | WebSocket: control in, telemetry out |
| `GET /stream.mjpg` | Video, `multipart/x-mixed-replace` |
| `GET /healthz` | Liveness |
| `GET /manifest.webmanifest`, `/icon-*.png`, `/favicon.svg` | PWA assets |

## How it reaches the vehicle

The car is on a **Tailscale** network, and the page is published over HTTPS with
`tailscale serve`. Three reasons, and only the first is convenience:

1. It works from any network, so the campus wifi's client isolation stops mattering.
2. **The browser's speech recognition and microphone APIs require a secure context.**
   Over plain HTTP they silently do not exist, so push-to-talk cannot work at all without
   HTTPS. A self-signed certificate is not enough for a phone, which makes `tailscale serve`
   the shortest honest path to a trusted certificate on a private network.
3. There is no authentication in the app. Access control is membership of the tailnet, and
   nothing else. **That is the security model, so understand it before copying this onto an
   open network**: anyone who can reach port 8080 can drive the car.

Without Tailscale the page still works over plain HTTP on a local network. The sticks,
buttons and video are unaffected. Only browser speech stops working, and the on-board
microphone path is unaffected because that never involved the browser.

## The protocol

JSON objects over one WebSocket, each with a `type`. Nothing is stateful: every message
stands alone, so a dropped connection loses nothing but time.

### Phone to robot

| `type` | Fields | Becomes |
|---|---|---|
| `teleop` | `throttle`, `steering` | `/teleop_cmd` (Twist) |
| `arm` | `armed` | `/arm` (Bool) |
| `estop` | `engaged` | `/estop` (Bool) |
| `command` | `action`, `seconds` | `/voice_cmd` (JSON String) |
| `voice` | `text`, `alternatives`, `source` | `/voice_cmd` (JSON String) |
| `lock` | `engage` | `/target_lock` (JSON String) |
| `pan` | `deg` | `/pan_manual` (Float32) |
| `safety` | `enabled` | `/safety_enabled` (Bool) |
| `autonomy` | `enabled` | `/autonomy_enabled` (Bool) |
| `video` | `config` | `/video_config` (JSON String) |
| `color` | `config` | `/color_config` (JSON String) |
| `ping` | `t` | echoed back as `pong`, for the latency readout |

### Robot to phone, pushed at 5 Hz

| `type` | From | Carries |
|---|---|---|
| `target` | `/target_state` | lock, status, offset, distance, confidence |
| `behavior` | `/behavior_state` | the running primitive and sequence progress |
| `arbiter` | `/arbiter/status` | armed or not |
| `obstacle` | `/obstacle_state` | guard enabled, what it is blocking for, sector ranges |
| `pan` | `/pan_state` | camera angle, and whether the axis is alive |
| `condition` | `/condition_state` | colour currently seen |
| `pong` | n/a | round-trip latency |

Telemetry is pushed at 5 Hz on purpose, well under perception's ~12 Hz. An operator is
reading it, not controlling on it, and it crosses a phone link.

### Video

A separate MJPEG stream rather than frames over the WebSocket, so a slow video connection
cannot delay a stick movement or a KILL. The frames come from the tracker's passthrough,
which is why video runs at the detector's rate and not the camera's.

## What the phone cannot do

The page is a **claimant, not a controller**. It sends requests; the robot decides.

- `arbiter_node` is the sole publisher of `/cmd_vel`, and it ranks the e-stop above the
  sticks above autonomy. The phone cannot bypass that ordering.
- Every command path has a **staleness timeout**. If the phone stops sending, the car stops.
  Silence is a valid stop, which covers a dropped connection, a locked screen and a crashed
  browser identically.
- Nothing moves until **ARM**, and the last client disconnecting disarms the vehicle and
  re-enables the LiDAR guard after a grace period.
- The camera is arbitrated too: `behavior_node` is the sole publisher of `/pan_cmd`, and the
  phone's slider is one claimant among follow tracking and spoken look commands.

## Speech, and where it happens

Two independent paths, on purpose:

- **On-board microphone** into `stt_node`, offline, using Vosk with a constrained grammar.
  No network, no browser, no phone. This is the primary path.
- **Browser speech** from the phone's push-to-talk, which posts a transcript plus its
  alternatives to `/voice_cmd`. Requires HTTPS, and only Chromium and Safari implement it;
  Firefox has never shipped `SpeechRecognition`.

Both produce the same message, so everything downstream is identical. Anything the offline
keyword vocabulary cannot parse is escalated to a self-hosted LLM, which runs on a machine on
the same network rather than a cloud API.
