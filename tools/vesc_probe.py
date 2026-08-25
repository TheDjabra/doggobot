#!/usr/bin/env python3
"""Talk to the VESC directly, with ROS out of the picture.

When the car will not move, the question is always "is it the software stack or
the motor controller?". This answers that by opening the VESC itself, reading
its telemetry, and commanding RPM step by step while reading back what the motor
actually did.

The car must be on a stand with the wheels free.

Nothing else may hold /dev/ttyACM0 while this runs, so stop the drive stack
first:  pkill -f vesc_twist_node
"""
import sys
import time

from pyvesc import VESC

PORT = '/dev/ttyACM0'

# VESC mc_fault_code enum, the values that actually show up in practice.
FAULTS = {
    0: 'NONE', 1: 'OVER_VOLTAGE', 2: 'UNDER_VOLTAGE', 3: 'DRV',
    4: 'ABS_OVER_CURRENT', 5: 'OVER_TEMP_FET', 6: 'OVER_TEMP_MOTOR',
    7: 'GATE_DRIVER_OVER_VOLTAGE', 8: 'GATE_DRIVER_UNDER_VOLTAGE',
    9: 'MCU_UNDER_VOLTAGE', 10: 'BOOTING_FROM_WATCHDOG_RESET',
    11: 'ENCODER_SPI', 12: 'ENCODER_SINCOS_BELOW_MIN_AMPLITUDE',
    13: 'ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE', 14: 'FLASH_CORRUPTION',
    15: 'HIGH_OFFSET_CURRENT_SENSOR_1', 16: 'HIGH_OFFSET_CURRENT_SENSOR_2',
    17: 'HIGH_OFFSET_CURRENT_SENSOR_3', 18: 'UNBALANCED_CURRENTS',
}


def show(v, label):
    m = v.get_measurements()
    if m is None:
        print(f'  {label}: no telemetry returned')
        return None
    fault = getattr(m, 'mc_fault_code', None)
    print(f'  {label}: rpm={getattr(m, "rpm", None)} '
          f'duty={getattr(m, "duty_cycle_now", None)} '
          f'motor_A={getattr(m, "avg_motor_current", None)} '
          f'input_A={getattr(m, "avg_input_current", None)} '
          f'V_in={getattr(m, "v_in", None)} '
          f'temp_fet={getattr(m, "temp_fet", None)} '
          f'fault={FAULTS.get(fault, fault)}')
    return m


def main():
    print(f'opening {PORT} ...')
    with VESC(serial_port=PORT, has_sensor=False, start_heartbeat=True,
              baudrate=115200) as v:
        print('connected')
        try:
            print('firmware:', v.get_firmware_version())
        except Exception as e:
            print('firmware read failed:', e)

        print('\n--- baseline, motor idle ---')
        base = show(v, 'idle')
        if base is not None and getattr(base, 'v_in', 0) is not None:
            vin = base.v_in
            if vin < 6.0:
                print(f'\n  !! V_in is {vin} V. The VESC is running on USB power '
                      'with no main battery, which powers the logic and the servo '
                      'but CANNOT drive the motor. This alone explains a silent '
                      'motor with working steering.')

        print('\n--- RPM sweep (each step 3 s, then coast) ---')
        for rpm in (500, 1000, 2000, 3000):
            print(f'commanding {rpm} ERPM')
            for _ in range(30):          # hold the command: the VESC times out
                v.set_rpm(int(rpm))      # on stale commands and stops the motor
                time.sleep(0.1)
            show(v, f'  at {rpm}')
            v.set_rpm(0)
            time.sleep(1.5)

        print('\n--- duty-cycle fallback (5%%), in case RPM mode is the problem ---')
        for _ in range(30):
            v.set_duty_cycle(0.05)
            time.sleep(0.1)
        show(v, '  duty 0.05')
        v.set_duty_cycle(0.0)
        v.set_rpm(0)

        print('\ndone')


if __name__ == '__main__':
    sys.exit(main())
