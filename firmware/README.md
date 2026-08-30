# Firmware

The camera pan axis uses a **Feetech STS3215 serial-bus servo** through a bus adapter, driven by
an ESP32. Not PWM: it is half-duplex TTL UART at 1 Mbps, with an absolute magnetic encoder that
reports actual position back.

That read-back is the reason for choosing it. Steering error while the camera is panned is
`pan_angle + bbox_offset_angle`, and with a hobby PWM servo `pan_angle` is only the value you
commanded, with nothing confirming the servo got there.

## Bring-up first, mechanics later

`esp32_sts_bringup/` is an interactive console: scan the bus, ping, read position and load, move,
set IDs, store a zero offset. Prove the electrical and firmware path on the bench before anything
is bolted to the car, so a failure is one variable rather than five.

Ported from the AI TURRET's `teensy_sts_bringup.ino`, which drives the same servos through the
same adapter. The port is the UART pin mapping and nothing else.

## Wiring

| adapter | ESP32 |
|---|---|
| GND | GND — **mandatory**, or the UART floats |
| RX | GPIO 17 (TX) |
| TX | GPIO 16 (RX) |

Board mode jumper on **UART**, not USB. Servo power **7.4 V into the adapter**, never from the
ESP32; only the grounds are shared. A 1000 µF capacitor across the servo rail absorbs stall
spikes. If `scan` finds nothing, suspect swapped TX/RX first.

**Pin choice matters**: on the classic ESP32, `Serial1`'s default pins are wired to the flash
chip, and using them unremapped fails in ways that look like bad wiring. 16/17 are safe.

## Things the turret already learned about these servos

- Servos ship as **ID 1**. Set IDs with **one servo on the bus at a time**.
- 0 to 4095 counts is one full turn; 2048 is centre. Default bus baud 1 Mbps.
- `zero` writes a calibration offset to EEPROM and survives power cycles: loosen the axis with
  `torque 0`, hold it where home should be, then `zero`.
- **Clamp every commanded position.** Pan free-spins, and on 2026-07-15 an unclamped target walked
  the turret's axis into the count ceiling and tangled its wiring. A camera on a cable makes that
  worse, and the reacquisition sweep is exactly that motion.

## What the ESP32 should and should not do

**Should**: accept "go to angle X", report actual angle back, clamp to a safe window.

**Should not**: arbitrate between competing requests. Three things will want the pan axis — manual
"look right", the follow controller keeping a target centred, and the reacquisition sweep — and
that is the same multi-publisher problem that produced `arbiter_node` for the drive axis. The
arbitration belongs on the Pi, in one place. An ESP32 that also tries to be clever gives you two
arbiters disagreeing.
