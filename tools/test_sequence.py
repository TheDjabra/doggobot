#!/usr/bin/env python3
"""Publish a sequence to behavior_node, and watch it run.

Nested JSON through ssh -> docker exec -> bash -> `ros2 topic pub` needs four
layers of quoting and silently degrades into something that looks like a plain
text command. This publishes with rclpy directly instead, so the message is
exactly what was intended.

  docker exec Doggobot python3 .../tools/test_sequence.py
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

SEQUENCE = [
    {'action': 'forward', 'seconds': 2},
    {'action': 'circle_left', 'seconds': 3},
    {'action': 'reverse', 'seconds': 2},
    {'action': 'stop'},
]


def main():
    rclpy.init()
    node = Node('sequence_tester')
    cmd = node.create_publisher(String, 'voice_cmd', 10)
    arm = node.create_publisher(Bool, 'arm', 10)
    seen = []
    node.create_subscription(
        String, 'behavior_state',
        lambda m: seen.append(json.loads(m.data)), 10)

    def wait_for_subscriber(pub, name, timeout=10.0):
        """ROS2 discovery is asynchronous: publishing before the far side has
        connected sends the message nowhere, silently. Sleeping a fixed second
        works most of the time, which is worse than failing consistently. Wait
        for an actual subscriber instead."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            if pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(node, timeout_sec=0.05)
        print(f'  WARNING: nothing subscribed to {name} after {timeout:.0f}s')
        return False

    wait_for_subscriber(arm, '/arm')
    wait_for_subscriber(cmd, '/voice_cmd')

    print('arming')
    arm.publish(Bool(data=True))
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    print('sending sequence:', ' -> '.join(s['action'] for s in SEQUENCE))
    cmd.publish(String(data=json.dumps(
        {'action': 'sequence', 'steps': SEQUENCE, 'source': 'test'})))

    t0 = time.time()
    last = None
    while time.time() - t0 < 14:
        rclpy.spin_once(node, timeout_sec=0.1)
        if seen and seen[-1] != last:
            last = seen[-1]
            print(f'  t={time.time() - t0:5.1f}s  active={last.get("active")}  '
                  f'step {last.get("step")}/{last.get("steps")}  '
                  f'waiting_for={last.get("waiting_for")}')

    print('disarming')
    arm.publish(Bool(data=False))
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
