// esp32_sts_bringup.ino - STS3215 serial-bus servo bring-up console for ESP32.
//
// Ported from the AI TURRET's teensy_sts_bringup.ino, which drives the same
// servos through the same adapter. The port is two lines: the ESP32 needs its
// UART pins named explicitly, where the Teensy has them fixed.
//
// Type commands in the Serial Monitor @115200 (newline line ending).
//
// WIRING (adapter <-> ESP32):
//   adapter GND -> ESP32 GND        common ground, MANDATORY or the UART floats
//   adapter TX  -> ESP32 GPIO 17   VERIFIED WORKING 2026-08-30
//   adapter RX  -> ESP32 GPIO 16
//   (i.e. RX_PIN = 17, TX_PIN = 16 below. The naming reads backwards because
//   RX_PIN is the pin the ESP32 RECEIVES on, which is wired to the adapter's TX.
//   Getting this the other way round produces a silent bus and nothing else.)
//   Board mode jumper on UART (not USB).
//   Servo power 7.4 V into the adapter's DC jack or screw terminal, NEVER from
//   the ESP32. Only the grounds are shared between the two power domains.
//   If `scan` finds nothing, suspect swapped TX/RX before anything else.
//
// WHY NAME THE PINS: on the classic ESP32, Serial1's default pins are wired to
// the flash chip, and using them unremapped fails in ways that look like bad
// wiring. The S3's map differs, but naming them explicitly is correct either way.
//
// VERIFIED HARDWARE (2026-08-29): the board on hand is an ESP32-S3 (dual core +
// LP, 240 MHz, 8 MB PSRAM, 16 MB flash) behind an FT232R bridge, so it appears
// as /dev/ttyUSB* rather than ttyACM*.
//
// CHECK 16/17 AGAINST YOUR BOARD'S PINOUT before wiring. S3 dev boards vary, and
// some route particular GPIOs to onboard peripherals (RGB LED, PSRAM on octal
// parts). Any free pair works; change RX_PIN/TX_PIN to match.
//
// A 1000 uF capacitor across the servo power rail absorbs stall spikes; the
// turret needed it and this will too.
//
// Commands:
//   scan            ping IDs 1..15, list what is on the bus (also runs at boot)
//   ping <id>       is this ID alive?
//   stat <id>       position / speed / load / voltage / temp / moving
//   move <id> <pos> go to absolute position 0..4095 (2048 = centre), gently
//   nudge <id> <d>  move by +/- d counts from current position
//   center <id>     go to 2048
//   id <old> <new>  change a servo's bus ID (EEPROM; survives power cycle)
//                   !! ONE servo on the bus at a time. They all ship as ID 1.
//   torque <id> <0|1>  torque off (free-spin by hand) / on
//   zero <id>       store the CURRENT physical position as count 2048 (EEPROM).
//                   Loosen (torque 0), hold the axis where you want home, THEN
//                   zero it.
//   sweep <id>      slow sweep across the safe window, to watch it move
//   help
//
// STS3215 facts: 0..4095 counts = 360 deg (2048 = centre), default bus baud
// 1 Mbps, default ID 1. Speed unit = counts/s, ACC = counts/s^2 * 100.

#include <SCServo.h>

const char* FW_VERSION = "0.1.0-esp32";

// ESP32 UART pins for the servo bus. See WHY THESE PINS above.
const int RX_PIN = 17;
const int TX_PIN = 16;
const uint32_t BUS_BAUD = 1000000;   // STS3215 factory default

// Gentle defaults for bring-up: nothing is mounted yet, so crawl.
const u16 MOVE_SPEED = 300;   // counts/s (~26 deg/s)
const u8  MOVE_ACC   = 20;    // soft ramp; 0 = no limit, avoid

// Clamp every commanded position. The turret learned this the hard way on
// 2026-07-15: pan free-spins, and an unclamped target walked the axis into the
// count ceiling and tangled the wiring. A camera on a cable makes that worse.
const int SAFE_LO = 1024;     // 2048 +/- 90 degrees
const int SAFE_HI = 3072;

SMS_STS st;

int clampPos(int p) {
  if (p < SAFE_LO) return SAFE_LO;
  if (p > SAFE_HI) return SAFE_HI;
  return p;
}

void printHelp() {
  Serial.println(F("commands: scan | ping <id> | stat <id> | move <id> <0-4095>"));
  Serial.println(F("          nudge <id> <+/-counts> | center <id> | sweep <id>"));
  Serial.println(F("          id <old> <new> | torque <id> <0|1> | zero <id> | help"));
  Serial.print(F("safe window: ")); Serial.print(SAFE_LO);
  Serial.print(F(" to ")); Serial.println(SAFE_HI);
}

void doScan() {
  Serial.println(F("scanning IDs 1..15 ..."));
  int found = 0;
  for (int id = 1; id <= 15; id++) {
    if (st.Ping(id) != -1) {
      Serial.print(F("  found ID ")); Serial.print(id);
      Serial.print(F("  pos=")); Serial.print(st.ReadPos(id));
      Serial.print(F("  volts=")); Serial.println(st.ReadVoltage(id) / 10.0, 1);
      found++;
    }
    delay(20);
  }
  if (!found) {
    Serial.println(F("  nothing on the bus."));
    Serial.println(F("  check: TX/RX swapped? jumper on UART? servo powered?"));
    Serial.println(F("  common ground between adapter and ESP32?"));
  }
}

void doStat(int id) {
  if (st.Ping(id) == -1) { Serial.println(F("no reply")); return; }
  Serial.print(F("id ")); Serial.print(id);
  Serial.print(F("  pos "));    Serial.print(st.ReadPos(id));
  Serial.print(F("  speed "));  Serial.print(st.ReadSpeed(id));
  Serial.print(F("  load "));   Serial.print(st.ReadLoad(id));
  Serial.print(F("  volts "));  Serial.print(st.ReadVoltage(id) / 10.0, 1);
  Serial.print(F("  temp "));   Serial.print(st.ReadTemper(id));
  Serial.print(F("  moving ")); Serial.println(st.ReadMove(id));
}

void doSweep(int id) {
  Serial.println(F("sweeping; watch it move. any key stops."));
  int targets[] = {2048, SAFE_LO + 200, 2048, SAFE_HI - 200, 2048};
  for (int i = 0; i < 5; i++) {
    if (Serial.available()) { Serial.read(); break; }
    st.WritePosEx(id, clampPos(targets[i]), MOVE_SPEED, MOVE_ACC);
    delay(1800);
  }
  Serial.println(F("done"));
}

void handle(char* line) {
  char cmd[16] = {0};
  int a = 0, b = 0;
  int n = sscanf(line, "%15s %d %d", cmd, &a, &b);
  if (n < 1) return;

  if (!strcmp(cmd, "scan"))                        doScan();
  else if (!strcmp(cmd, "help"))                   printHelp();
  else if (!strcmp(cmd, "ping")   && n >= 2)       Serial.println(st.Ping(a) == -1 ? F("no reply") : F("alive"));
  else if (!strcmp(cmd, "stat")   && n >= 2)       doStat(a);
  else if (!strcmp(cmd, "sweep")  && n >= 2)       doSweep(a);
  else if (!strcmp(cmd, "center") && n >= 2)     { st.WritePosEx(a, 2048, MOVE_SPEED, MOVE_ACC); Serial.println(F("centering")); }
  else if (!strcmp(cmd, "move")   && n >= 3)     { int p = clampPos(b); st.WritePosEx(a, p, MOVE_SPEED, MOVE_ACC); Serial.print(F("moving to ")); Serial.println(p); }
  else if (!strcmp(cmd, "nudge")  && n >= 3)     { int p = clampPos(st.ReadPos(a) + b); st.WritePosEx(a, p, MOVE_SPEED, MOVE_ACC); Serial.print(F("moving to ")); Serial.println(p); }
  else if (!strcmp(cmd, "torque") && n >= 3)     { st.EnableTorque(a, b ? 1 : 0); Serial.println(b ? F("torque on") : F("torque off (free-spin)")); }
  else if (!strcmp(cmd, "id")     && n >= 3)     { st.unLockEprom(a); st.writeByte(a, SMS_STS_ID, b); st.LockEprom(b); Serial.print(F("id ")); Serial.print(a); Serial.print(F(" -> ")); Serial.println(b); }
  else if (!strcmp(cmd, "zero")   && n >= 2)     { st.CalibrationOfs(a); Serial.println(F("current position stored as 2048")); }
  else                                             Serial.println(F("?  type help"));
}

void setup() {
  Serial.begin(115200);
  // THE PORT: the ESP32 needs its UART pins named. Everything else is identical
  // to the Teensy version.
  Serial1.begin(BUS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  st.pSerial = &Serial1;

  delay(600);
  Serial.print(F("\nSTS3215 bring-up v")); Serial.print(FW_VERSION);
  Serial.print(F(" (ESP32, bus on Serial1 @"));
  Serial.print(BUS_BAUD); Serial.println(F(")"));
  printHelp();
  doScan();
}

void loop() {
  static char buf[64];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handle(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    }
  }
}
