#!/usr/bin/env python3
"""Exercise the LLM slow path with utterances the keyword parser cannot handle.

  docker exec Doggobot bash -c "source .../env.sh && python3 .../tools/test_llm.py"
"""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

CASES = [
    'drive forward for five seconds then spin left twice',
    'go until you see the green thing and then stop',
    'back up a little bit and then do a figure eight',
    'make me a sandwich',                       # must come back not understood
]


def main():
    rclpy.init()
    node = Node('llm_tester')
    pub = node.create_publisher(String, 'voice_unparsed', 10)
    seen = []
    node.create_subscription(String, 'llm_state',
                             lambda m: seen.append(json.loads(m.data)), 10)

    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0:
        print('llm_node is not subscribed to /voice_unparsed. Is it running?')
        return 1

    for text in CASES:
        print(f'\n>>> {text!r}')
        seen.clear()
        pub.publish(String(data=json.dumps({'text': text, 'source': 'test'})))
        t0 = time.time()
        while time.time() - t0 < 20:
            rclpy.spin_once(node, timeout_sec=0.1)
            if seen and seen[-1]['state'] not in ('thinking',):
                s = seen[-1]
                print(f'    {s["state"]}  {s.get("detail", "")}  '
                      f'({time.time() - t0:.1f}s)')
                break
        else:
            print('    timed out with no response')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
