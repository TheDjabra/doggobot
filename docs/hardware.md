# Hardware

## Vehicle

1/10 scale car on the UCSD robocar platform.

| Item | Detail |
|---|---|
| Compute | Raspberry Pi 5, 16 GB, Raspberry Pi OS Bookworm (64-bit) |
| Motor control | VESC over USB CDC-ACM, `/dev/ttyACM*` |
| Camera | Luxonis OAK-D Lite, Myriad X accelerator, stereo baseline 7.5 cm |
| LiDAR | LD06, fitted 2026-08-26, `/dev/ttyUSB0` (CP2102 bridge) |
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

### Bench bring-up, verified 2026-08-30

Done on the Victus with a spare 2S 2200 mAh pack, before anything was mounted, because the
cheapest place to find a wiring fault is on a desk.

| Check | Result |
|---|---|
| Bus scan | `found ID 1  pos=1  volts=7.8` |
| Commanded 0, +/-30, +/-60 deg | every angle reached, worst error 1.1 deg |
| Settle time | 0.50 s for 30 deg, 1.00 s for a 120 deg swing, so roughly 120 deg/s |
| Bus read failures | 0 across the whole run |
| Supply under load | 7.8 to 7.9 V, no measurable sag |
| Temperature | 28 to 29 C |

**Verified pin mapping.** The first scan found nothing on the bus, and the cause was the first
suspect on the list below: TX and RX crossed.

| Adapter | ESP32-S3 |
|---|---|
| TX | GPIO **17** |
| RX | GPIO **16** |
| GND | GND |

The naming reads backwards on purpose. `RX_PIN` is the pin the ESP32 receives on, so it wires
to the adapter's transmit. Reproduce the check any time with `tools/pan_console.py --selftest`,
which reports measured-against-commanded error rather than declaring success on a write that
returned no error.

The USB bridge is an FTDI FT232R, serial **A5069RR4**. That serial is what
`deploy/99-doggobot-serial.rules` matches on, so the device gets a stable name instead of
racing the LiDAR and the VESC for `/dev/ttyUSB0`.

### The board on the car (2026-09-01)

A second ESP32-S3, smaller, with **native USB** rather than an FT232R. Consequences:

- It presents as Espressif's own USB-Serial/JTAG unit, `303a:1001`, serial
  `10:20:BA:0D:FA:C8` (the chip MAC), and enumerates as **ttyACM**, not ttyUSB.
- The firmware must be built with **`CDCOnBoot=cdc`**. A stock build leaves `Serial` on
  the hardware UART and the USB port stays silent with no error.
- `/dev/ttyACM0` is the **VESC**. So the pan axis is never found by probing ports:
  merely opening the VESC's tty asserts DTR and can reset it. `pan_node` resolves the
  device from sysfs by USB serial, which opens nothing, and then confirms with a banner.
- The udev symlink exists on the host but **not inside the container**, which gets its
  own `/dev`. A hotplugged device needs a `docker restart` to appear; a recreate is not
  required, and `tools/setup_container.sh` re-applies what a restart discards.
- ModemManager runs on this Pi and probes new ttyACM devices for a modem, so the udev
  rule sets `ID_MM_DEVICE_IGNORE`.

### Travel ceiling: +/-90 degrees

The pan axis must never go more than **90 degrees either side of centre**
(operator instruction, 2026-09-01). It is enforced independently at four layers, so
no single mistake can widen it:

| Layer | Mechanism |
|---|---|
| `esp32_pan` firmware | `ABS_LIMIT_DEG`, plus a `static_assert` that makes widening the working clamp a **build failure** |
| `pan_node` | clamps `limit_deg` to 90 and logs an error if the config asks for more |
| `behavior_node` | manual look and the phone slider clamp to the same ceiling |
| `tools/pan_console.py` | the bench self-test clamps its own angles rather than trusting the firmware |

Working clamps sit well inside it: firmware 80, `pan_node` 75, follow cascade 70.
The servo's encoder covers a full turn and **the axis free-spins**, which is why the
ceiling is enforced in firmware rather than only in configuration: a crashed host, a
bad parameter or a garbled serial line all stop at the same wall.

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

The servo is 7.4 V and the car's main pack is **4S** (14.8 V nominal, 16.8 V charged), so it
does not run off the main pack. A separate 2S pack feeds the adapter, sized for stall current,
which is amps rather than milliamps on a 19 kg servo.

(This section said 3S until 2026-08-30. That is the same wrong number that was set in VESC
Tool, which put the low-voltage cutoff at 2.25 V per cell. Every team on this course got
4S. Corrected here so the document cannot repeat the mistake.)

## LiDAR: role and mounting (decided 2026-08-26)

**Role: the 290 degrees the camera cannot see.** Not sensor fusion. The OAK-D already returns
fused XYZ per bounding box computed on-camera, so fusing a LiDAR range into the same target
would buy marginal precision at 1 to 5 m in exchange for a camera-to-LiDAR extrinsic
calibration that can be silently wrong. Spring 2023 Team 10 documented being burned by exactly
that mounting-offset problem. The camera covers roughly 70 degrees; the LiDAR covers the rest,
and that complement is where the value is.

Three cases in this project specifically:
- **Reversing.** The mission ends with "then reverse" and the camera faces forward, so nothing
  watches where the car is going.
- **Obstacles while following.** The car is steering to keep a person centred and is therefore,
  by construction, looking at the person and not at what it is about to clip.
- **During the pan-servo search sweep**, when the camera is deliberately pointed away.

**Implementation stays trivial**: `safety_node` subscribes `/scan`, takes the minimum range in
an angular window, publishes a stop on `/safety_cmd` when something is inside the threshold.
The refinement is that the **window depends on the behaviour's current state** (forward arc when
driving forward, rear arc when reversing, wide when searching), which the behaviour node already
knows. No SLAM, no costmap, no Nav2. Those are what make LiDAR expensive; a `min()` over ~450
floats at 10 Hz is not.

**Mounting: rigid, top of the stack, in the position originally intended for the GPS.**

- **GPS is dropped.** The RTK waypoint work is already graded and nothing in the final project
  uses it. Unplug the GNSS module (CP2105 dual bridge) to free USB power budget on the bus the
  OAK-D already peaks near 1 A on, and to stop it shuffling serial device enumeration.
- **LiDAR above the camera** is the correct order: the camera mount then sits below the scan
  plane and cannot occlude it. Reversed, there would be a permanent blind wedge rotating with
  the servo.
- **Known blind spot, accepted**: a high scan plane sees torsos and walls but not cones, curbs,
  or anything on the ground. Correct for the EBU courtyard hazards, and worth remembering so the
  safety node is not mistaken for full obstacle coverage.
- **Cable routing is the real risk.** A spinning LiDAR and a 270-degree panning camera will be
  adjacent. The turret already lost time to an unclamped pan axis tangling its wiring. Route the
  LiDAR cable out on the opposite side from the camera's sweep arc, strain-relieve both, and keep
  the software clamp on pan limits regardless.

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
