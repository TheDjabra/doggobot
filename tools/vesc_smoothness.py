#!/usr/bin/env python3
"""Find where the drivetrain stops being jumpy.

Commands a range of ERPM and, at each step, samples the ACTUAL rpm repeatedly.
A speed controller that cannot hold a setpoint at low rpm shows up as a large
spread between samples, which is what "jumpy" feels like from outside.

Reports mean, spread, and current draw per step, so the choice of operating
throttle is a measurement rather than a preference.

CAR ON A STAND. Stop the stack first: nothing else may hold /dev/ttyACM0.

  docker exec Doggobot python3 .../tools/vesc_smoothness.py
"""
import statistics
import time

from pyvesc import VESC

PORT = '/dev/ttyACM0'
MAX_RPM = 7640          # what vesc_twist_node computes: 0.382 * 20000
STEPS = [0.10, 0.13, 0.16, 0.20, 0.25, 0.30, 0.363, 0.382]


def main():
    with VESC(serial_port=PORT, has_sensor=False, start_heartbeat=True,
              baudrate=115200) as v:
        print(f'{"throttle":>9} {"ERPM cmd":>9} {"rpm mean":>9} {"spread":>8} '
              f'{"motor A":>8}  verdict')
        for thr in STEPS:
            target = int(MAX_RPM * thr)
            for _ in range(12):                 # spin up and settle
                v.set_rpm(target)
                time.sleep(0.1)

            rpms, amps = [], []
            for _ in range(20):                 # 2 s of samples while holding
                v.set_rpm(target)
                m = v.get_measurements()
                if m is not None and getattr(m, 'rpm', None) is not None:
                    rpms.append(m.rpm)
                    amps.append(m.avg_motor_current)
                time.sleep(0.1)

            v.set_rpm(0)
            time.sleep(0.8)

            if not rpms:
                print(f'{thr:9.3f} {target:9d}        no telemetry')
                continue

            mean = statistics.mean(rpms)
            spread = max(rpms) - min(rpms)
            amp = statistics.mean(amps) if amps else 0.0
            # Spread as a fraction of the target is the useful figure: 200 rpm of
            # wobble is nothing at 3000 and is the whole signal at 800.
            frac = spread / target if target else 0
            verdict = ('JUMPY' if frac > 0.35 else
                       'rough' if frac > 0.15 else 'smooth')
            print(f'{thr:9.3f} {target:9d} {mean:9.0f} {spread:8.0f} '
                  f'{amp:8.2f}  {verdict} ({frac * 100:.0f}% of target)')

        v.set_rpm(0)
        print('\ndone')


if __name__ == '__main__':
    main()
