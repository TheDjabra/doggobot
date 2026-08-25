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
