#!/usr/bin/env python3
"""Follow controller: /target_state -> /behavior_cmd.

Two independent loops over two numbers:

    steering  PD on  x       (bbox centre offset, -1..1, 0 = centred)
    throttle  PD on  z_mm    against a standoff setpoint

Nothing here knows about pixels, frame size, or the camera. perception_node
publishes `x` as an error term precisely so this file cannot care.

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
from std_msgs.msg import String


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
        self.declare_parameter('throttle_deadband_mm', 150.0)
        self.declare_parameter('throttle_floor', 0.13)
        self.declare_parameter('max_throttle', 0.25)
        self.declare_parameter('allow_reverse', True)

        self.declare_parameter('target_timeout_s', 0.5)

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
        self.timeout = float(g('target_timeout_s').value)

        self.last_stamp = None
        self.last_msg_time = 0.0
        self.last_steer = 0.0
        self.state = None

        self.cmd_pub = self.create_publisher(Twist, 'behavior_cmd', 10)
        self.create_subscription(String, 'target_state', self._on_target, 10)

        # A watchdog so that perception dying is not mistaken for "target
        # centred". Without it the last command would simply stop being updated.
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            f'follow: standoff {self.standoff:.0f} mm, '
            f'steer PD {g("kp_steer").value}/{g("kd_steer").value}, '
            f'throttle floor {self.floor}, max {self.max_throttle}')

    # -- helpers --------------------------------------------------------------

    def _step_over_floor(self, value):
        """Below the motor's deadband the car stops rather than creeping."""
        if value == 0.0:
            return 0.0
        mag = max(self.floor, min(self.max_throttle, abs(value)))
        return math.copysign(mag, value)

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
            return                       # silent: arbiter times out and stops

        stamp = float(t.get('stamp', time.time()))
        dt = (stamp - self.last_stamp) if self.last_stamp else 0.08
        self.last_stamp = stamp

        if status == 'LOST':
            # Keep the lock and keep pointing, but do not drive toward something
            # we cannot currently see.
            cmd = Twist()
            cmd.angular.z = self.last_steer
            cmd.linear.x = 0.0
            self.cmd_pub.publish(cmd)
            self.state = status
            return

        if status not in ('TRACKED', 'NEW'):
            self.state = status
            return

        # --- steering: PD on centre offset ---
        x = float(t.get('x', 0.0))
        steer = 0.0 if abs(x) < self.steer_deadband else self.steer_pid.step(x, dt)
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
                throttle = self._step_over_floor(raw)
            else:
                self.thr_pid.reset()

        cmd = Twist()
        cmd.linear.x = float(throttle)
        cmd.angular.z = float(steer)
        self.cmd_pub.publish(cmd)

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
