#!/usr/bin/env python3
"""Follow controller: /target_state -> /follow_cmd + /follow_pan.

A CASCADE, not two controllers arguing over one error. Each loop closes on a
different error and they run an order of magnitude apart in speed:

    inner   x            -> pan servo   keep the target centred IN FRAME
    outer   bearing      -> steering    point the CHASSIS where the camera looks
            z_mm         -> throttle    against a standoff setpoint

The camera chases the target and the car chases the camera. When the pan angle
returns to zero the car is aimed at the target by construction. This is gaze
stabilisation on a mobile base, the same structure as a turret except the base
moves too.

The point of it is that the target can no longer leave the frame during a turn.
With a fixed camera the tracking error and the car's turning radius were the same
loop, so a tight turn in a small room lost the lock. Now keeping the target in
frame is the inner loop's only job and it is not coupled to the chassis at all.

The bearing is the whole trick:

    bearing_deg = pan_deg + x * half_fov_deg      angle to target, chassis frame
    err_steer   = bearing_deg / half_fov_deg      normalised, so gains carry over

Note what that reduces to when pan_deg is 0: `err_steer == x`, exactly the
fixed-camera controller this file used to be. So a dead pan axis, an unplugged
ESP32, or `use_pan: false` all degrade to the old behaviour with the old tuning
rather than to a special case that only gets exercised when something is broken.

THE STALE-ANGLE TRAP. `x` describes a frame captured ~100 ms ago; the pan angle
read now is not the angle the camera had when that frame was taken. At a 120 deg/s
slew that is 12 degrees of pure fiction, which is larger than the errors being
controlled. Combining them naively produces a loop that looks badly tuned and
cannot be fixed by tuning. Hence the ring buffer: the pan angle is looked up at
the frame's own timestamp.

Nothing here knows about pixels or frame size. perception_node publishes `x` as
an error term precisely so this file cannot care.

Behaviour by target status:

    TRACKED  full control
    LOST     hold the last steering, cut throttle. The tracker's LOST state is a
             grace period for a brief occlusion, not a reason to abandon a lock,
             but driving blind toward a target we cannot see is not acceptable
             either. So: stop moving, keep pointing, keep the lock.
    other    publish nothing at all, and let the arbiter's staleness timeout
             release the car. Silence is a safer stop than a stream of zeros,
             because it also covers this node crashing.

Two hardware facts shape the throttle law, both measured (docs/hardware.md):

**The motor has a deadband.** Below linear.x ~0.13 the car does not creep, it
stops. A PD loop approaching its setpoint naturally commands ever-smaller
throttle, so without stepping over the floor the car would stall short of the
target every time and look broken. Hence `throttle_floor`.

**Steering is only clamped by us.** The class `vesc_twist_node` computes
`max_left_steering`/`max_right_steering` and never applies them, so the arbiter's
clamp is the only limit. We stay well inside it.
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


class PID:
    """Discrete PD(+I) on an explicit dt, with a derivative low-pass.

    The derivative term is filtered because the input arrives at ~12 Hz from a
    detector, and raw frame-to-frame differences of a bounding-box centre are
    mostly noise. An unfiltered D on that signal produces steering chatter.
    """

    def __init__(self, kp, ki, kd, d_alpha=0.4, i_limit=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.d_alpha, self.i_limit = d_alpha, i_limit
        self.prev_err = None
        self.integral = 0.0
        self.d_filt = 0.0

    def reset(self):
        self.prev_err = None
        self.integral = 0.0
        self.d_filt = 0.0

    def step(self, err, dt):
        if dt <= 0.0:
            dt = 1e-3
        d = 0.0
        if self.prev_err is not None:
            raw_d = (err - self.prev_err) / dt
            self.d_filt = self.d_alpha * raw_d + (1 - self.d_alpha) * self.d_filt
            d = self.d_filt
        self.prev_err = err

        if self.ki:
            self.integral = max(-self.i_limit,
                                min(self.i_limit, self.integral + err * dt))

        return self.kp * err + self.ki * self.integral + self.kd * d


class FollowNode(Node):

    def __init__(self):
        super().__init__('follow_node')

        # Steering gains start from the car's own lane-following calibration
        # (ros_racer_calibration.yaml: Kp 0.2 / Ki 0.0 / Kd 0.1). Same servo,
        # same geometry, same actuator scale, so they are a real starting point.
        # The error SOURCE differs though: a bbox centre at 12 fps is noisier
        # than a lane centroid, which is why the D term is filtered.
        self.declare_parameter('kp_steer', 0.2)
        self.declare_parameter('ki_steer', 0.0)
        self.declare_parameter('kd_steer', 0.1)
        self.declare_parameter('steer_deadband', 0.05)
        self.declare_parameter('max_steer', 0.7)

        self.declare_parameter('standoff_mm', 1000.0)
        self.declare_parameter('kp_throttle', 0.0004)
        self.declare_parameter('kd_throttle', 0.0001)
        self.declare_parameter('throttle_deadband_mm', 300.0)
        self.declare_parameter('throttle_floor', 0.365)
        self.declare_parameter('max_throttle', 0.25)
        self.declare_parameter('allow_reverse', False)
        # Reverse gets its own ceiling. Approaching a target and backing away
        # from one are not symmetric situations: someone stepping toward the car
        # produces a large negative error fast, and full-authority reverse there
        # reads as the car bolting rather than yielding.
        self.declare_parameter('max_reverse', 0.16)

        # -- pan cascade --
        # half_fov_deg converts the normalised bbox offset into an angle. It is
        # NOT the camera's spec-sheet HFOV: `x` is normalised across the
        # DETECTOR's input, and DetectionNetwork.build requests that input with
        # the default resize mode, CROP, so the network sees a centre crop of the
        # sensor and x = 1 spans a much narrower angle than the frame edge.
        # MEASURED at 25.2 on 2026-09-02 against a spec-derived 34.5, a 36% error.
        # tools/measure_fov.py reproduces it in about 20 seconds.
        self.declare_parameter('use_pan', True)
        self.declare_parameter('half_fov_deg', 25.2)
        self.declare_parameter('kp_pan', 0.85)
        self.declare_parameter('kd_pan', 0.05)
        # A moving person is a RAMP, and a proportional loop has a standing
        # error on a ramp: the camera settles a fixed angle behind them. The old
        # incremental form hid this by acting as a crude integrator, which is
        # also why it oscillated. Anchoring fixed the oscillation and exposed
        # the lag, so the integrator goes back in deliberately and bounded.
        self.declare_parameter('ki_pan', 0.0)
        self.declare_parameter('i_limit_pan', 0.6)
        self.declare_parameter('pan_limit_deg', 70.0)
        self.declare_parameter('pan_deadband', 0.04)
        # Resync the integrated pan command to the measured angle if they drift
        # apart: that means a stall, a slipped mount, or someone holding it.
        self.declare_parameter('pan_slip_deg', 15.0)
        # false restores the older incremental form, kept only so the two can be
        # compared rather than argued about.
        self.declare_parameter('pan_anchor_to_measured', True)
        self.declare_parameter('pan_stale_s', 0.5)
        # perception_node stamps /target_state at PUBLISH time, not capture time,
        # so the frame is already this old when the stamp is written. The pan
        # history is looked up at (stamp - capture_lag_s) to compensate. Measure
        # it rather than guessing: it is the gap between the OAK's frame clock
        # and the publish, and it is the difference between a loop that tracks
        # and one that hunts. 0.0 disables the correction.
        self.declare_parameter('capture_lag_s', 0.0)
        # Above this bearing, stop driving and turn first. A car cannot strafe,
        # so creeping forward while the target is off to one side only makes the
        # geometry worse. behavior_node escalates to a three-point turn.
        self.declare_parameter('align_before_drive_deg', 50.0)

        self.declare_parameter('target_timeout_s', 0.5)
        self.declare_parameter('debug_hz', 0.0)   # >0 logs decisions at that rate

        g = self.get_parameter
        self.steer_pid = PID(float(g('kp_steer').value),
                             float(g('ki_steer').value),
                             float(g('kd_steer').value))
        self.thr_pid = PID(float(g('kp_throttle').value), 0.0,
                           float(g('kd_throttle').value))

        self.steer_deadband = float(g('steer_deadband').value)
        self.max_steer = float(g('max_steer').value)
        self.standoff = float(g('standoff_mm').value)
        self.thr_deadband = float(g('throttle_deadband_mm').value)
        self.floor = float(g('throttle_floor').value)
        self.max_throttle = float(g('max_throttle').value)
        self.allow_reverse = bool(g('allow_reverse').value)
        self.max_reverse = float(g('max_reverse').value)
        self.timeout = float(g('target_timeout_s').value)
        self.debug_hz = float(g('debug_hz').value)
        self._last_debug = 0.0

        self.use_pan = bool(g('use_pan').value)
        self.half_fov = float(g('half_fov_deg').value)
        self.kp_pan = float(g('kp_pan').value)
        self.kd_pan = float(g('kd_pan').value)
        self.pan_limit = min(float(g('pan_limit_deg').value), 90.0)  # ceiling
        self.pan_deadband = float(g('pan_deadband').value)
        self.pan_slip = float(g('pan_slip_deg').value)
        self.pan_anchor = bool(g('pan_anchor_to_measured').value)
        self.pan_stale = float(g('pan_stale_s').value)
        self.capture_lag = float(g('capture_lag_s').value)
        self.align_deg = float(g('align_before_drive_deg').value)

        # Output is an angular CORRECTION in units of x; scaled to degrees by
        # half_fov at the call site, then integrated onto the commanded angle.
        self.pan_pid = PID(self.kp_pan, float(g('ki_pan').value), self.kd_pan,
                           i_limit=float(g('i_limit_pan').value))
        self.pan_cmd = 0.0            # what we last asked for (integrated)
        self.pan_hist = []            # (stamp, measured_deg) ring buffer
        self.pan_ok = False
        self.pan_last_msg = 0.0

        self.last_stamp = None
        self.last_msg_time = 0.0
        self.last_steer = 0.0
        self.state = None

        # NOT /behavior_cmd: behavior_node owns that topic and relays this one
        # when follow is the active primitive. Two publishers on one command
        # topic is the same race the arbiter exists to prevent, one layer up.
        self.cmd_pub = self.create_publisher(Twist, 'follow_cmd', 10)
        # NOT /pan_cmd: pan_node has one writer and behavior_node is it, the same
        # arbitration rule that keeps this node off /behavior_cmd.
        self.pan_pub = self.create_publisher(Float32, 'follow_pan', 10)
        self.create_subscription(String, 'target_state', self._on_target, 10)
        self.create_subscription(String, 'pan_state', self._on_pan, 10)

        # A watchdog so that perception dying is not mistaken for "target
        # centred". Without it the last command would simply stop being updated.
        self.create_timer(0.1, self._watchdog)

        # A floor above a ceiling would make the clamp emit MORE than the stated
        # maximum, silently. Catch it at startup rather than in the field.
        for name, ceiling in (('max_throttle', self.max_throttle),
                              ('max_reverse', self.max_reverse)):
            if self.floor > ceiling:
                self.get_logger().error(
                    f'throttle_floor {self.floor} exceeds {name} {ceiling}: the '
                    f'floor will override the ceiling. Raise {name}.')

        # LIVE TUNING. Every parameter here was cached at startup, so
        # `ros2 param set` changed the parameter and nothing else: that cost a
        # calibration run earlier today, when use_pan was set false and the node
        # carried on regardless. Gains are the ones worth iterating on with the
        # car in front of you, so they now take effect immediately.
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f'follow: standoff {self.standoff:.0f} mm, '
            f'steer PD {g("kp_steer").value}/{g("kd_steer").value}, '
            f'throttle floor {self.floor}, max {self.max_throttle}')

    def _on_set_params(self, params):
        """Apply gain changes at runtime, so tuning does not need a restart."""
        live = {
            'kp_steer': lambda v: setattr(self.steer_pid, 'kp', v),
            'ki_steer': lambda v: setattr(self.steer_pid, 'ki', v),
            'kd_steer': lambda v: setattr(self.steer_pid, 'kd', v),
            'kp_pan':   lambda v: setattr(self.pan_pid, 'kp', v),
            'kd_pan':   lambda v: setattr(self.pan_pid, 'kd', v),
            'ki_pan':   lambda v: setattr(self.pan_pid, 'ki', v),
            'kp_throttle': lambda v: setattr(self.thr_pid, 'kp', v),
            'kd_throttle': lambda v: setattr(self.thr_pid, 'kd', v),
            'max_steer': lambda v: setattr(self, 'max_steer', v),
            'steer_deadband': lambda v: setattr(self, 'steer_deadband', v),
            'half_fov_deg': lambda v: setattr(self, 'half_fov', v),
            'standoff_mm': lambda v: setattr(self, 'standoff', v),
            'use_pan': lambda v: setattr(self, 'use_pan', v),
            'pan_limit_deg': lambda v: setattr(self, 'pan_limit', min(v, 90.0)),
            'pan_anchor_to_measured': lambda v: setattr(self, 'pan_anchor', v),
            'align_before_drive_deg': lambda v: setattr(self, 'align_deg', v),
        }
        for p in params:
            fn = live.get(p.name)
            if fn is not None:
                fn(p.value)
                self.get_logger().info(f'{p.name} -> {p.value}')
        try:
            from rcl_interfaces.msg import SetParametersResult
            return SetParametersResult(successful=True)
        except ImportError:                                  # tests, no ROS
            return type('R', (), {'successful': True})()

    # -- helpers --------------------------------------------------------------

    def _step_over_floor(self, value):
        """Below the motor's deadband the car stops rather than creeping."""
        if value == 0.0:
            return 0.0
        ceiling = self.max_throttle if value > 0 else self.max_reverse
        mag = max(self.floor, min(ceiling, abs(value)))
        return math.copysign(mag, value)

    def _on_pan(self, msg):
        """Keep a short history of measured pan angles, tagged by arrival time.

        A ring buffer rather than a single latest value, because the follow loop
        needs the angle the camera HAD when a frame was captured, not the angle
        it has now. See the stale-angle trap in the module docstring.
        """
        try:
            d = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return
        self.pan_last_msg = time.time()
        self.pan_ok = bool(d.get('ok')) and d.get('deg') is not None
        if not self.pan_ok:
            return
        self.pan_hist.append((float(d.get('stamp', time.time())), float(d['deg'])))
        if len(self.pan_hist) > 60:          # ~3 s at 20 Hz, far more than needed
            self.pan_hist = self.pan_hist[-60:]

    def _pan_at(self, stamp):
        """Measured pan angle at a past instant, or None if the axis is not live."""
        if not self.pan_enabled():
            return None
        if not self.pan_hist:
            return None
        best = min(self.pan_hist, key=lambda e: abs(e[0] - stamp))
        # If the closest sample is far from the frame, the history is not usable
        # and guessing is worse than declaring the axis unavailable.
        if abs(best[0] - stamp) > 0.5:
            return None
        return best[1]

    def pan_enabled(self):
        return (self.use_pan and self.pan_ok
                and (time.time() - self.pan_last_msg) < self.pan_stale)

    def _watchdog(self):
        if self.state is None:
            return
        if time.time() - self.last_msg_time > self.timeout:
            if self.state != 'STALE':
                self.get_logger().warn('target_state went stale, releasing')
                self.state = 'STALE'
                self.steer_pid.reset()
                self.thr_pid.reset()
            # Publish nothing: the arbiter's own timeout stops the car.

    # -- control --------------------------------------------------------------

    def _on_target(self, msg):
        try:
            t = json.loads(msg.data)
        except Exception:                                    # noqa: BLE001
            return

        self.last_msg_time = time.time()
        status = t.get('status')

        if not t.get('locked'):
            if self.state != status:
                self.state = status
                self.steer_pid.reset()
                self.thr_pid.reset()
                self.pan_pid.reset()
                # Do NOT recentre the camera here. behavior_node decides where it
                # points when nothing is being followed, because a search sweep
                # also wants the axis and this node is no longer the only claimant.
            return                       # silent: arbiter times out and stops

        stamp = float(t.get('stamp', time.time()))
        dt = (stamp - self.last_stamp) if self.last_stamp else 0.08
        self.last_stamp = stamp

        if status == 'LOST':
            # Keep the lock and keep pointing, but do not drive toward something
            # we cannot currently see. The camera holds its angle rather than
            # recentring: the target was last seen there, so that is the best
            # place to be looking when it reappears.
            cmd = Twist()
            cmd.angular.z = self.last_steer
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            self.state = status
            return

        if status not in ('TRACKED', 'NEW'):
            self.state = status
            return

        x = float(t.get('x', 0.0))

        # --- inner loop: aim the camera ---
        # Integrated on the LAST COMMANDED angle, not the measured one. The servo
        # takes ~0.5 s to settle while frames arrive every 80 ms, so correcting
        # from the measured angle would repeatedly re-issue corrections the servo
        # is still in the middle of executing, and the axis would crawl.
        # Commanded is what we actually control; the slip check below is what
        # keeps that from drifting away from reality.
        pan_meas = self._pan_at(stamp - self.capture_lag)
        if pan_meas is not None:
            # A large gap between what was asked for and what the servo
            # reached still means something is wrong (a stall, a slipped horn,
            # someone holding it), so it is still worth saying out loud. It no
            # longer needs a resync: every command is anchored to the
            # measurement already.
            if abs(self.pan_cmd - pan_meas) > self.pan_slip:
                self.get_logger().warn(
                    f'pan not reaching its target: asked {self.pan_cmd:+.0f}, '
                    f'measured {pan_meas:+.0f}')
            # Step the PD every frame even inside the deadband, so its
            # derivative history stays continuous; only the OUTPUT is gated.
            # A D term fed intermittently differentiates the gaps as well as the
            # signal and kicks on the first sample after a quiet stretch.
            correction = self.pan_pid.step(x, dt) * self.half_fov
            if abs(x) >= self.pan_deadband:
                # ABSOLUTE target from the angle the camera HAD when the frame
                # was taken, not an increment onto the last command.
                #
                # Incrementing overshoots by construction. The servo takes about
                # half a second to settle and frames arrive every 80 ms, so six
                # corrections stack up before the first has finished executing,
                # each computed from an x that the camera has not moved to
                # correct yet. Anchoring to the measured angle instead means
                # successive frames during one move all ask for roughly the SAME
                # place, and the target can never lie beyond the target's true
                # bearing, so the axis converges onto it rather than past it.
                base = pan_meas if self.pan_anchor else self.pan_cmd
                self.pan_cmd = max(-self.pan_limit,
                                   min(self.pan_limit, base + correction))
            self.pan_pub.publish(Float32(data=float(self.pan_cmd)))

        # --- bearing: where the target is relative to the CHASSIS ---
        # With no pan axis this is x * half_fov, so err_steer collapses to x and
        # the original fixed-camera controller is recovered exactly.
        pan_for_bearing = pan_meas if pan_meas is not None else 0.0
        bearing = pan_for_bearing + x * self.half_fov
        err_steer = bearing / self.half_fov

        # --- outer loop: point the chassis ---
        steer = (0.0 if abs(err_steer) < self.steer_deadband
                 else self.steer_pid.step(err_steer, dt))
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # --- throttle: PD on distance against the standoff ---
        z = float(t.get('z_mm', 0))
        throttle = 0.0
        if z > 0:
            err = z - self.standoff            # positive = too far, go forward
            if abs(err) > self.thr_deadband:
                raw = self.thr_pid.step(err, dt)
                if raw < 0 and not self.allow_reverse:
                    raw = 0.0
                # Badly misaligned: turn first. Driving forward at a target 60
                # degrees off the nose closes very little of the actual gap and
                # swings the bearing further out. Above the threshold, stop; below
                # it, scale by cos so speed falls off smoothly as alignment goes.
                if abs(bearing) > self.align_deg:
                    raw = 0.0
                else:
                    raw *= max(0.0, math.cos(math.radians(bearing)))
                throttle = self._step_over_floor(raw) if raw else 0.0
            else:
                self.thr_pid.reset()

        cmd = Twist()
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)

        # Log the inputs and the outputs from the SAME message. Sampling
        # /target_state and /cmd_vel separately gives skewed pairs that look like
        # control bugs and are not.
        if self.debug_hz > 0:
            now = time.time()
            if now - self._last_debug >= 1.0 / self.debug_hz:
                self._last_debug = now
                err_s = (f'{z - self.standoff:+7.0f}' if z > 0
                         else '  no-depth')
                pan_s = (f'{pan_meas:+5.1f}' if pan_meas is not None else '  off')
                self.get_logger().info(
                    f'x={x:+.3f} pan={pan_s} brg={bearing:+6.1f} '
                    f'z={z:6.0f} err={err_s} '
                    f'-> steer={steer:+.3f} thr={throttle:+.3f}')

        self.last_steer = steer
        if self.state != status:
            self.get_logger().info(f'following id {t.get("id")}')
        self.state = status


def main(args=None):
    rclpy.init(args=args)
    node = FollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
