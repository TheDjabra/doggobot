# Architecture

## The one integration point

The car already runs the UCSD robocar ROS2 framework (`ucsd_robocar_hub2`). The entire seam
between this project and that framework is a single topic: **`geometry_msgs/Twist` on
`/cmd_vel`**, consumed by `vesc_twist_node` in `ucsd_robocar_actuator2_pkg`. `linear.x` is
throttle, `angular.z` is steering. Publish valid Twist there and the car moves.

Nothing in the framework is forked or modified. This project adds nodes alongside it. Reading
across prior teams in the course GitHub org, the teams that extended the framework this way
consistently did better than the teams that forked it.

DonkeyCar is not used. DonkeyCar is the behavioral-cloning path from an earlier assignment
(record frames, train a CNN, let it drive). Every prior voice-controlled and target-following
team in the org also built ROS2 packages rather than extending DonkeyCar.

## The single-publisher rule

**Exactly one node publishes `/cmd_vel`.** Two publishers on one topic is not an error in
ROS2, it is a race: the actuator receives interleaved messages from both at whatever rate
each happens to run, so a "stop" and a "follow" alternate and the car twitches. Every other
behavior publishes to its own intermediate topic and an arbiter node decides.

```
voice_bridge_node   -> /voice_cmd       (FastAPI + rclpy in one process)
perception_node     -> /target_state    (DepthAI spatial detection + ObjectTracker)
behavior_node       -> /behavior_cmd    (primitives + the sequencing executor)
safety_node         -> /safety_cmd      (LiDAR proximity, e-stop)
                            |
                        arbiter_node -> /cmd_vel     <- sole publisher
```

Arbiter priority, highest first: e-stop, then safety override, then the active primitive.
Its timer runs at 10 to 20 Hz and publishes on **every** tick, not only on change, because
the base controller times out on missing `/cmd_vel` messages and zeroes the motors. That
timeout is a safety property: the point of streaming commands is the failure case, when a
publisher crashes or the network drops.

## Node responsibilities

**`voice_bridge_node`** serves the phone web app and turns input into structured commands.
One process running FastAPI and `rclpy` together: `rclpy.init()`, a node holding the
publishers, spun in a background thread, with the WebSocket handler calling into it. A
separate rosbridge server was considered and rejected, because a server process is needed
anyway to serve the page over HTTPS, proxy the LLM call so the key stays out of the browser,
and run the parser.

**`perception_node`** runs the DepthAI pipeline: ColorCamera preview into a
`YoloSpatialDetectionNetwork` with StereoDepth linked, into an `ObjectTracker`, out over
XLink. The spatial network returns per-detection XYZ in millimetres computed on the camera,
so no depth math happens on the Pi. The tracker assigns persistent IDs with
NEW/TRACKED/LOST/REMOVED states, which is what "lock onto a target" means here: no hand-rolled
association logic, and the LOST state is a free grace period before declaring the target gone.

**`behavior_node`** owns the primitives, the vocabulary, and the sequencing executor, and is the
sole publisher of `/behavior_cmd`. Primitives: forward, reverse, circle_left, circle_right, wait,
stop, and follow. Follow is a primitive rather than a parallel system, so mutual exclusion is
structural: `follow_node` publishes `/follow_cmd` and this node relays it only while follow is
the active mode.

Every moving primitive is time-bounded. "Forward until further notice" is how a car ends up in a
wall when a link drops.

The executor runs an ordered list of steps. Spoken chains ("forward then circle left then stop")
are a keyword split, which covers the common case without an LLM's latency or network dependency.
A chain where any step fails to parse is refused entirely rather than partially executed. Steps
may carry an `until` condition read from `/condition_state`; nothing publishes that yet, and the
step's duration doubles as a timeout so a condition that never arrives cannot strand the car.

**`arbiter_node`** is the sole `/cmd_vel` publisher, as above.

**Threading note:** perception and control stay in separate nodes because the default `rclpy`
executor runs one callback at a time with no preemption. A vision callback that overruns
delays the control timer by exactly its overrun.

## Message types

`std_msgs/String` carrying JSON on the intermediate topics; `geometry_msgs/Twist` only at
the arbiter output. Custom ROS2 message definitions require a CMake package, which is
friction with no payoff at this scale.

## Control

**PID only.** Steering is a PD loop on the tracklet's horizontal offset from frame center.
Throttle is a PD loop on measured distance against a setpoint. LQR and LQG are deliberately
out of scope: state-space control needs a model of a plant you understand, and this vehicle
was not built by us. The class framework is PID-shaped already
(`Kp_steering`/`Ki_steering`/`Kd_steering`), so this also means less to fight.

Kalman filtering remains available as an optional smoothing layer on the target estimate,
not as a controller, and only with evidence it is needed: the ObjectTracker already runs its
own Kalman filter on-camera.

## Voice input

Two tiers, for latency and for robustness:

- **Fast path**: fixed commands (stop, wait, follow, circle, figure-8, loop) matched by
  keyword on the phone with no model call. Near-instant.
- **Slow path**: arbitrary sequences go to an LLM that emits a schema-constrained list of
  primitives from a fixed vocabulary.

The fast path alone runs the entire graded demo, so a failure of the LLM tier or its network
dependency degrades the system rather than stopping it.

Speech is captured in the browser on the phone, not on the car. This keeps speech-to-text
compute off the Pi entirely (only short text strings cross the network) and keeps the
microphone at the speaker's mouth rather than on a vehicle that is driving away.

## Target reacquisition

The camera is on a pan servo. On target loss the system holds a short grace period, then
sweeps the camera to search for the target, re-locks, and resumes. This exists because a
prior team documented reacquisition-after-loss as an unsolved problem in their own project,
where the robot simply stopped when the target left frame.

Because the camera pans independently of the chassis, steering error is the sum of the
servo's own angle and the target's offset within the frame, not the offset alone. Likewise,
the depth reading is a range along the camera's optical axis: forward distance is
`Z*cos(pan)` and lateral offset is `Z*sin(pan)`.

## Close-range framing

The camera pans but does not tilt, so a person at close range overflows the vertical field
of view and the detector sees a torso crop, exactly when the car is nearest and least able
to recover from a lost lock. Mitigations: drive the loop on horizontal offset only, never
gate on box height or aspect ratio, use asymmetric confidence thresholds (strict to acquire
a lock, lenient to hold one), and set the follow standoff above the distance where framing
degrades rather than fighting it.
