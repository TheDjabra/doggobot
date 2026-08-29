#!/usr/bin/env python3
"""Find which way the LiDAR thinks "forward" is.

Mounting rotation is a physical fact nobody writes down, and getting it wrong
means the guard watches the wrong side of the car: it would stop for a wall
behind while driving into one ahead.

Runs CONTINUOUSLY rather than taking one snapshot, because a snapshot cannot
tell your test object apart from the furniture. Wave something in front of the
car and watch which direction tracks it.

  docker exec Doggobot bash -c "source .../env.sh && python3 .../tools/lidar_sectors.py [seconds]"

Then set `forward_offset_deg` in config/safety.yaml to the angle it reports while
the object is directly ahead.
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

BUCKETS = 24                      # 15 degrees each
WIDTH = 2 * math.pi / BUCKETS


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
    rclpy.init()
    node = Node('lidar_sectors')
    latest = {}
    node.create_subscription(LaserScan, 'scan', lambda m: latest.update(scan=m), 10)

    print('Hold an object about 0.4 m from the car and move it around the front.')
    print('Watch which angle follows it. Ctrl-C when you are sure.\n')

    t0 = time.time()
    last = 0.0
    while time.time() - t0 < seconds and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        scan = latest.get('scan')
        if scan is None or time.time() - last < 0.4:
            continue
        last = time.time()

        best = [None] * BUCKETS
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < max(scan.range_min, 0.02) or r > scan.range_max:
                continue
            a = (scan.angle_min + i * scan.angle_increment) % (2 * math.pi)
            b = int(a / WIDTH) % BUCKETS
            if best[b] is None or r < best[b]:
                best[b] = r

        bi, br = None, 1e9
        for b, r in enumerate(best):
            if r is not None and r < br:
                bi, br = b, r
        if bi is None:
            print('  nothing in range')
            continue

        centre = math.degrees((bi + 0.5) * WIDTH)
        signed = centre if centre <= 180 else centre - 360
        # A coarse compass so the direction is readable at a glance.
        ring = ['.'] * BUCKETS
        ring[bi] = '#'
        print(f'  nearest {br:5.2f} m at {centre:5.0f} deg '
              f'(offset {signed:+4.0f})   [{"".join(ring)}]')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
