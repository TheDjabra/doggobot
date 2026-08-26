# Doggobot

Voice-commanded autonomous robocar. MAE/ECE 148 Team 4, UC San Diego, Summer Session 2 2026.

Cesar Montiel (ECE), Hektoras Djabra (MAE), Naveen Weedagama (MAE).

## What it does

Speak to the car from a phone and it executes the command: simple behaviors immediately
(stop, wait, follow, circle, figure-8), and multi-step sequences gated on what the camera
sees ("go to the red, turn right, go to the green, then reverse").

Mission statement from the proposal: drive forward until green is detected, turn around,
continue until red is detected.

## Architecture in one paragraph

The car runs ROS2. Every behavior publishes to its own topic and exactly one node, the
arbiter, publishes `geometry_msgs/Twist` on `/cmd_vel`, which is the only seam with the
class `ucsd_robocar_hub2` framework. A phone web app captures speech in the browser and
pushes recognized text over a WebSocket to a FastAPI service on the car, so no speech
processing runs on the Pi. Perception runs on the OAK-D Lite's own accelerator, which
returns bounding boxes with fused stereo depth, so following a person is a PD loop on
pixel offset for steering and measured distance for throttle. The camera sits on a
serial-bus servo, so when a target is lost the camera sweeps to reacquire it instead of
the car simply stopping.

Detail in [docs/architecture.md](docs/architecture.md). Hardware in
[docs/hardware.md](docs/hardware.md). Day-by-day record in [build_log.md](build_log.md).

## Status

Drivable from a phone. `arbiter_node` and `voice_bridge_node` are implemented and validated on
hardware, including the deadman and e-stop paths. Perception, behaviour primitives, the
sequencing executor, and speech recognition are not built yet.

## Layout

```
doggobot/        ROS2 python package (nodes go here)
launch/          launch files
config/          YAML parameters
docs/            design and hardware documentation
build_log.md     dated engineering record
```

## Running it

Runs inside the `Doggobot` container on the car's Raspberry Pi 5.
**[docs/runbook.md](docs/runbook.md)** is the step-by-step from `ssh doggobot` to driving from
a phone. Hardware detail in [docs/hardware.md](docs/hardware.md).

## License

MIT.
