#!/usr/bin/env python3
"""Bench tool for the camera pan axis. Talks the esp32_pan line protocol.

    ./pan_console.py --selftest          command angles and verify it got there
    ./pan_console.py p 30 c              send commands, print the reply stream
    ./pan_console.py                     interactive

The self-test exists because "the servo moved" and "the servo went where it was
told" are different claims, and only the second one is worth anything to the
control loop above it. So it reports measured-vs-commanded error and settle time
rather than declaring success on a write that returned no error.
"""
import argparse
import sys
import time

import serial


class Pan:
    def __init__(self, port, baud=115200, timeout=0.05):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2.0)              # ESP32 resets on DTR; wait out the reboot
        self.ser.reset_input_buffer()
        self.deg = None
        self.target = None
        self.moving = 0
        self.volts = 0.0
        self.temp = 0
        self.errs = 0
        self.buf = ''

    def send(self, cmd):
        self.ser.write((cmd.strip() + '\n').encode())

    def pump(self, seconds, echo=False):
        """Read for `seconds`, parsing status lines into state."""
        end = time.time() + seconds
        while time.time() < end:
            chunk = self.ser.read(256).decode('utf-8', 'replace')
            if not chunk:
                continue
            self.buf += chunk
            while '\n' in self.buf:
                line, self.buf = self.buf.split('\n', 1)
                line = line.strip()
                if not line:
                    continue
                if line.startswith('#'):
                    if echo:
                        print(line)
                    continue
                if line.startswith('s '):
                    p = line.split()
                    if len(p) >= 8:
                        try:
                            self.deg = float(p[1]) if p[1] != 'nan' else None
                            self.target = float(p[2])
                            self.moving = int(p[3])
                            self.volts = float(p[5])
                            self.temp = int(p[6])
                            self.errs = int(p[7])
                        except ValueError:
                            pass
                    if echo:
                        print(line)

    def goto(self, deg, tol=1.5, timeout=3.0):
        """Command an angle, wait for it, return (reached, error, seconds)."""
        self.send(f'p {deg}')
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.05)
            if self.deg is not None and abs(self.deg - deg) <= tol:
                return True, self.deg - deg, time.time() - t0
        err = (self.deg - deg) if self.deg is not None else float('nan')
        return False, err, time.time() - t0


def selftest(pan):
    pan.pump(1.0, echo=True)
    if pan.deg is None:
        print('\nFAIL: no position reported. Servo unpowered, or bus silent.')
        return 1

    print(f'\nstart: {pan.deg:+.1f} deg, {pan.volts:.1f} V, {pan.temp} C, '
          f'{pan.errs} bus errors')
    print('\n  target   reached   error   settle')
    print('  ------   -------   -----   ------')

    ok = True
    for angle in (0.0, 30.0, -30.0, 60.0, -60.0, 0.0):
        reached, err, secs = pan.goto(angle)
        flag = '' if reached else '   <-- MISSED'
        got = f'{pan.deg:+.1f}' if pan.deg is not None else ' nan'
        print(f'  {angle:+6.1f}   {got:>7}   {err:+5.1f}   {secs:5.2f}s{flag}')
        ok &= reached
        time.sleep(0.2)

    pan.pump(0.3)
    print(f'\nend: {pan.volts:.1f} V, {pan.temp} C, {pan.errs} bus errors')
    if pan.errs > 2:
        print('WARNING: bus read failures. Check power sag or wiring.')
    print('\nPASS: pan axis tracks commanded angle.' if ok else
          '\nFAIL: at least one angle was not reached.')
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/ttyUSB0')
    ap.add_argument('--selftest', action='store_true')
    ap.add_argument('cmds', nargs='*')
    a = ap.parse_args()

    try:
        pan = Pan(a.port)
    except serial.SerialException as e:
        print(f'cannot open {a.port}: {e}')
        return 2

    if a.selftest:
        return selftest(pan)

    if a.cmds:
        pan.pump(0.5, echo=True)
        for c in a.cmds:
            print(f'> {c}')
            pan.send(c)
            pan.pump(1.5, echo=True)
        return 0

    print('type commands (p <deg>, v <deg/s>, e 0|1, c, ?, i), ctrl-c to quit')
    try:
        while True:
            pan.pump(0.2, echo=True)
            # non-blocking-ish: only read stdin when the user has typed
            import select
            if select.select([sys.stdin], [], [], 0.0)[0]:
                pan.send(sys.stdin.readline())
    except KeyboardInterrupt:
        print()
    return 0


if __name__ == '__main__':
    sys.exit(main())
