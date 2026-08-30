#!/usr/bin/env python3
"""Read the pack voltage before the stack claims the VESC.

Runs as a systemd ExecStartPre, in the one window where it can: the class
`vesc_twist_node` holds /dev/ttyACM0 exclusively and publishes no telemetry, so
once the stack is up nothing can read the battery. Before it starts, the port is
free.

Exits 0 always. A battery warning must never prevent the car from starting; the
point is to tell you the number, not to make the decision for you.
"""
import sys

CELLS = 4                 # 4S pack
NOMINAL = 3.7
LOW = 3.5                 # per cell: get a fresh pack before a demo
CRITICAL = 3.2            # per cell: charge now, driving from here risks damage


def main():
    try:
        from pyvesc import VESC
    except ImportError:
        print('battery: pyvesc unavailable, skipping')
        return 0

    try:
        with VESC(serial_port='/dev/ttyACM0', has_sensor=False,
                  start_heartbeat=False, baudrate=115200) as v:
            m = v.get_measurements()
            vin = float(m.v_in)
    except Exception as e:                                   # noqa: BLE001
        print(f'battery: could not read VESC ({e})')
        return 0

    per = vin / CELLS
    if per < CRITICAL:
        verdict = 'CRITICAL - charge before driving'
    elif per < LOW:
        verdict = 'LOW - fine to test, do not start a demo on this'
    elif per > 4.15:
        verdict = 'full'
    else:
        verdict = 'ok'
    print(f'battery: {vin:.2f} V pack, {per:.2f} V/cell ({CELLS}S) - {verdict}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
