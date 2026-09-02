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

| adapter | ESP32-S3 | |
|---|---|---|
| GND | GND | **mandatory**, or the UART floats |
| TX | GPIO **17** | verified working 2026-08-30 |
| RX | GPIO **16** | |

**Bring-up result (2026-08-30)**: `found ID 1  pos=1  volts=7.8`, then `center 1` drove it to
2048 with position, speed and load reported back throughout. Whole chain proven on the bench
before anything was mounted: ESP32-S3 -> UART @1 Mbps -> adapter -> STS3215, with encoder
feedback returning. Powered from a 2S pack reading 7.8 V at the adapter.

The first attempt found nothing on the bus, and the cause was exactly the first suspect in the
sketch's own error message: TX and RX crossed. Swapping them in firmware found the servo
immediately.

Board mode jumper on **UART**, not USB. Servo power **7.4 V into the adapter**, never from the
ESP32; only the grounds are shared. A 1000 µF capacitor across the servo rail absorbs stall
spikes. If `scan` finds nothing, suspect swapped TX/RX first.

**Pin choice matters**: on the classic ESP32, `Serial1`'s default pins are wired to the flash
chip, and using them unremapped fails in ways that look like bad wiring. Naming them explicitly is
correct on any variant.

**The board on hand (identified 2026-08-29) is an ESP32-S3**: dual core plus LP core at 240 MHz,
8 MB PSRAM, 16 MB flash, behind an **FT232R** bridge rather than the S3's native USB, so it
enumerates as `/dev/ttyUSB*` and not `ttyACM*`. **Check GPIO 16/17 against this board's own
pinout before wiring**. S3 dev boards vary and some route particular GPIOs to onboard
peripherals. Any free pair works; change `RX_PIN`/`TX_PIN` to match.

It arrived flashed with a **PWM servo sweep** (printing `us 600` … `us 2400`), which drives a
hobby servo. The STS3215 takes no PWM at all, so that firmware is for a different class of servo
and gets replaced by the bring-up sketch.

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

**Should not**: arbitrate between competing requests. Three things will want the pan axis: manual
"look right", the follow controller keeping a target centred, and the reacquisition sweep, and
that is the same multi-publisher problem that produced `arbiter_node` for the drive axis. The
arbitration belongs on the Pi, in one place. An ESP32 that also tries to be clever gives you two
arbiters disagreeing.

## esp32_pan: the run-mode firmware

`esp32_sts_bringup` is an interactive console for a human. `esp32_pan` is the one the robot
actually uses, and it is deliberately dumb: it sets angles and reports angles. It decides
nothing about where to look.

That split matters. A microcontroller that second-guesses its host is a second controller
fighting the first, which is the exact failure `arbiter_node` exists to prevent, relocated one
layer down where it is much harder to see.

### Protocol

Newline-terminated ASCII, 115200 baud, both directions.

| Host to ESP32 | Meaning |
|---|---|
| `p <deg>` | target angle, signed degrees, 0 is centre |
| `v <deg/s>` | slew speed limit |
| `e <0\|1>` | torque enable, 0 lets you move it by hand |
| `c` | centre |
| `?` | one status line now |
| `i` | identity banner |

The ESP32 streams status at 50 Hz:

```
s <deg> <target> <moving> <load> <volts> <temp> <errs>
```

Lines starting `#` are human-readable and carry no state. Angles cross this wire, never encoder
counts: that the STS3215 happens to put centre at 2048 is an implementation detail of this
file and nothing upstream should have to know it.

### Safety

A hard clamp at +/-80 degrees lives in the firmware as well as in `pan_node`, because this is
the layer that cannot be talked out of it. A bad angle from a ROS bug, a mistyped serial
command, or a garbled byte all stop at the same wall. This axis free-spins, and an unclamped
target has previously walked it into its own wiring.

### Power, and what drives the servo

The servo is **not** driven by the ESP32 and **not** powered from the vehicle's main pack.

```
  2S LiPo 7.4V ----> Waveshare Bus Servo Adapter (A) ----> STS3215
                              ^  jumper on A (UART-SERVO)
                              |  half-duplex TTL UART, 1 Mbps
                       ESP32-S3 GPIO 16/17 + GND
```

The adapter is the servo driver: it takes 7.4 V into its own screw terminal and converts the
ESP32's UART into the half-duplex bus the servo speaks. Three rules, each of which has cost
somebody an evening somewhere:

- **Servo power goes into the adapter**, never from the microcontroller's 5 V pin. A 19 kg
  servo's stall current is amps.
- **Common ground** between adapter and ESP32 is mandatory, or the UART floats.
- The main pack is 4S and the servo is 7.4 V, which is why this axis gets its own 2S pack
  rather than a tap off the vehicle.

### Toolchain

| | |
|---|---|
| arduino-cli core | `esp32:esp32` 3.3.11 |
| Library | `SCServo` 1.0.2 (provides `SMS_STS.h`) |

### Building and flashing

**The FQBN options are not optional on a native-USB board.** Ours enumerates as
`303a:1001`, Espressif's own USB-Serial/JTAG unit, rather than through a separate UART chip.
A stock `esp32:esp32:esp32s3` build leaves `Serial` on the hardware UART pins, so the USB
port stays **completely silent with no error at all**, which looks exactly like a dead board.

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc" \
    -u -p /dev/ttyACM0 esp32_pan
python3 ../tools/pan_console.py --port /dev/ttyACM0 --selftest
```

A board behind an external USB-UART chip (an FT232R, say) enumerates as `ttyUSB*` and does
not need `CDCOnBoot`, since its `Serial` already goes out over that chip.

### Flashing from the Pi, once it is on the vehicle

`arduino-cli` need not be installed on the Pi. Build the binaries on a laptop and push them:

```bash
# on the laptop
arduino-cli compile --fqbn "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc" \
    --output-dir /tmp/panbuild esp32_pan
scp /tmp/panbuild/esp32_pan.ino*.bin \
    ~/.arduino15/packages/esp32/hardware/esp32/*/tools/partitions/boot_app0.bin  pi@doggobot:~/panfw/

# on the Pi, once:  python3 -m venv ~/esptool-venv && ~/esptool-venv/bin/pip install esptool
cd ~/panfw && ~/esptool-venv/bin/esptool --chip esp32s3 --port /dev/doggobot-pan \
    --baud 460800 write-flash -z \
    0x0     esp32_pan.ino.bootloader.bin \
    0x8000  esp32_pan.ino.partitions.bin \
    0xe000  boot_app0.bin \
    0x10000 esp32_pan.ino.bin
```

If the board ever goes silent and stays silent, a hard reset recovers it:

```bash
~/esptool-venv/bin/esptool --port /dev/doggobot-pan --after hard-reset --no-stub chip-id
```

**Do not open its serial port with DTR asserted.** On a native-USB S3, DTR/RTS combinations
are how the chip is put into download mode, and pyserial asserts DTR by default, so a naive
reconnect loop will hold a working board offline indefinitely. `pan_node` sets `dtr` and
`rts` false before `open()` for exactly this reason.
