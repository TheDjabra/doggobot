# Hardware

## Vehicle

1/10 scale car on the UCSD robocar platform.

| Item | Detail |
|---|---|
| Compute | Raspberry Pi 5, 16 GB, Raspberry Pi OS Bookworm (64-bit) |
| Motor control | VESC over USB CDC-ACM, `/dev/ttyACM*` |
| Camera | Luxonis OAK-D Lite, Myriad X accelerator, stereo baseline 7.5 cm |
| LiDAR | LD06 (not fitted as of 2026-08-25) |
| Battery | **4S**, measured 15.1 V at the VESC |

### Measured drive characteristics (bench, 2026-08-25)

Read directly off the VESC with `tools/vesc_probe.py`, motor idle and under command.

| Quantity | Value |
|---|---|
| Firmware | 6.6.55 |
| Input voltage | 15.1 V (a 4S pack) |
| FET temperature | 26.6 C idle |
| Fault code | none, at idle and under load |
| Motor start threshold | **between 500 and 1000 ERPM.** 500 does not turn the motor, 1000 does |

How that maps to what you publish: `vesc_twist_node` computes
`max_rpm = max_throttle * max_rpm = 0.382 * 20000 = 7640` ERPM, then commands
`rpm = 7640 * linear.x`. So:

| `linear.x` | ERPM | Moves? |
|---|---|---|
| 0.05 | 382 | no, below the floor |
| 0.10 | 764 | no, below the floor |
| **0.13** | ~1000 | **the practical floor** |
| 0.20 | 1528 | yes |
| 0.363 to 0.382 | 2773 to 2918 | the class lap speed, what `lane_guidance_node` publishes |

**The consequence for control design**: throttle has a deadband, so a behavior slowing toward
a target cannot ramp smoothly to zero. Below roughly 0.13 the car does not creep, it stops.
Any follow or approach loop has to treat this as a floor and step over it.

**Battery note.** The pack is 4S and every team received the same. The vault's record of the
VESC motor wizard says it was configured with a **3S** Li-ion battery profile. If that is
still the case, the low-voltage cutoff is set for 3S and will not protect a 4S pack. Worth
confirming in VESC Tool; nothing is wrong at present (no faults, normal temperatures).

## Pan axis

| Item | Detail |
|---|---|
| Servo | Feetech STS3215, 7.4 V, 19 kg, magnetic encoder |
| Driver | Bus servo adapter (Waveshare / Wonrabai "Bus Servo Adapter (A)") |
| Power | Separate 2S pack into the adapter's own power input |

The STS3215 is a **serial-bus servo, not PWM**. It speaks half-duplex TTL UART at 1 Mbps by
default, with many servos addressable by ID on one wire. This matters because the VESC's
servo output is already committed to steering, so there was no spare PWM channel available.

It is also the better choice on its merits: a 12-bit absolute magnetic encoder reports actual
position at 0.088 degree resolution, along with load, voltage, and temperature, all read back
over the same bus. Steering error while the camera is panned depends on the servo's angle, and
with a hobby PWM servo that angle is the commanded value with nothing verifying the servo
reached it. Here it is measured. The load read-back is also how a binding cable gets caught
before it strips a gear during a sweep.

### Facts carried over from prior work with these servos

- Board jumper on **A / UART**. Adapter TX to host RX, adapter RX to host TX. If a bus scan
  finds nothing, suspect swapped TX/RX first.
- **Common ground** between the adapter and the host is mandatory.
- Servo power goes into the adapter, never from the host controller.
- 0 to 4095 counts is one full 360 degree turn; 2048 is center. Default bus baud 1 Mbps.
- Servos ship as **ID 1**. Set IDs with one servo on the bus at a time.
- A midpoint calibration writes an offset to EEPROM and survives power cycles.
- **The pan axis free-spins.** An unclamped target once walked the axis to the count ceiling
  and tangled the wiring. Clamp every commanded position, especially during a search sweep,
  which is exactly that failure mode with a camera cable attached.

### Power

The servo is 7.4 V and the car's main pack is 3S (11.1 V nominal, 12.6 V charged), so it does
not run off the main pack. A separate 2S pack feeds the adapter, sized for stall current,
which is amps rather than milliamps on a 19 kg servo.

## Microphone

Primary input is the phone's own microphone, with speech recognition running in the phone's
browser. A Samson Go Mic (USB, class-compliant, cardioid mode) stays plugged into the Pi as a
near-zero-cost fallback input, not built out as a second full pipeline.

## Known trap: OAK-D Lite power on the Pi

The camera can draw around 1 A peak at 5 V, and the peak scales with pipeline complexity.
RGB plus stereo plus a neural network plus the tracker is exactly the worst case. Under-current
drops the camera off the USB bus or brown-outs the Pi mid-run. Mitigations: a 5 V / 5 A supply
or `usb_max_current_enable=1` in `/boot/firmware/config.txt`, and a real USB3 cable in a USB3
port. On USB2, pin the link speed explicitly rather than letting it negotiate and flap.

## Container bring-up

Everything runs inside the `Doggobot` Docker container on the Pi.

```bash
ssh robocar
docker start Doggobot
docker exec -it Doggobot bash
source_ros2                 # run again after every build_ros2
```

For anything with a GUI, refresh the X11 cookie from the Pi host after each reconnect:

```bash
cat ~/.Xauthority | docker exec -i Doggobot sh -c "cat > /root/.Xauthority"
```

If a device disappears inside the container but exists on the host, the container's `/dev`
view has desynced after a USB re-enumeration. `docker restart Doggobot` refreshes it, then
re-copy the X11 cookie.
