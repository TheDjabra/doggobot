#!/usr/bin/env python3
"""Check that spoken chains parse into sequences, without needing a voice.

Publishes text exactly as the recognisers would, so the parsing can be verified
separately from pronunciation.

  docker exec Doggobot bash -c "source .../env.sh && python3 .../tools/test_chain.py"
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

CASES = [
    ('phone-speech', 'forward then circle left then stop'),
    ('phone-speech', 'circle right and then reverse'),
    ('onboard-mic', 'atlas forward then circle left'),
    ('onboard-mic', 'forward then stop'),          # no wake word: must be ignored
    ('phone-speech', 'forward then jump then stop'),  # unknown step: reject chain
]


def main():
    rclpy.init()
    node = Node('chain_tester')
    cmd = node.create_publisher(String, 'voice_cmd', 10)
    arm = node.create_publisher(Bool, 'arm', 10)

    t0 = time.time()
    while cmd.get_subscription_count() == 0 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)

    arm.publish(Bool(data=True))
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)

    for source, text in CASES:
        print(f'\n[{source}] "{text}"')
        cmd.publish(String(data=json.dumps({'text': text, 'source': source})))
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(1.5)
        # Cancel whatever started, so the next case begins clean.
        cmd.publish(String(data=json.dumps(
            {'action': 'stop', 'source': 'phone-button'})))
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

    arm.publish(Bool(data=False))
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
