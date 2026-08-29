#!/usr/bin/env python3
"""Find which way the LiDAR thinks "forward" is.

Mounting rotation is a physical fact nobody writes down, and getting it wrong
means the guard watches the wrong side of the car: it would stop for a wall
behind while driving into one ahead. Rather than guess, put an object directly
in front of the car and read off where it appears.

  docker exec Doggobot bash -c "source .../env.sh && python3 .../tools/lidar_sectors.py"

Then set `forward_offset_deg` in config/safety.yaml to the angle it reports.
"""
import math
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

BUCKETS = 24                      # 15 degrees each


def main():
    rclpy.init()
    node = Node('lidar_sectors')
    got = []
    node.create_subscription(LaserScan, 'scan', lambda m: got.append(m), 10)

    print('put an object ~0.5 m directly in FRONT of the car, then wait...')
    t0 = node.get_clock().now().nanoseconds
    while not got and (node.get_clock().now().nanoseconds - t0) < 10e9:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not got:
        print('no /scan received. Is the ldlidar node running?')
        return 1

    scan = got[-1]
    print(f'angle_min={math.degrees(scan.angle_min):.1f} deg  '
          f'angle_max={math.degrees(scan.angle_max):.1f} deg  '
          f'{len(scan.ranges)} points  '
          f'range {scan.range_min:.2f}-{scan.range_max:.1f} m\n')

    width = 2 * math.pi / BUCKETS
    best = [None] * BUCKETS
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < max(scan.range_min, 0.02) or r > scan.range_max:
            continue
        a = (scan.angle_min + i * scan.angle_increment) % (2 * math.pi)
        b = int(a / width) % BUCKETS
        if best[b] is None or r < best[b]:
            best[b] = r

    print(f'{"bucket":>18}  nearest')
    closest_bucket, closest = None, 1e9
    for b in range(BUCKETS):
        lo = math.degrees(b * width)
        hi = math.degrees((b + 1) * width)
        r = best[b]
        bar = '' if r is None else '#' * max(1, int(30 - min(r, 3.0) * 10))
        print(f'{lo:6.0f} to {hi:5.0f} deg  '
              f'{"-" if r is None else f"{r:5.2f} m"}  {bar}')
        if r is not None and r < closest:
            closest, closest_bucket = r, b

    if closest_bucket is not None:
        centre = math.degrees((closest_bucket + 0.5) * width)
        # Report as a signed offset, which is what the parameter wants.
        signed = centre if centre <= 180 else centre - 360
        print(f'\nnearest object at ~{centre:.0f} deg ({closest:.2f} m)')
        print(f'if that object is directly ahead, set  forward_offset_deg: {signed:.0f}')

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
