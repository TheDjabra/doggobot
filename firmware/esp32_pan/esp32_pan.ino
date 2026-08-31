// esp32_pan - run-mode firmware for the Doggobot camera pan axis.
//
// The bring-up sketch (../esp32_sts_bringup) is an interactive console for a
// human. This one is a line protocol for `pan_node.py`, and it is deliberately
// dumb: it sets angles and reports angles. It does NOT decide where to look.
//
// That split is the same rule the rest of the stack follows. arbiter_node is the
// sole /cmd_vel publisher; pan_node is the sole writer to this serial port; and
// this firmware arbitrates nothing at all. A microcontroller that second-guesses
// its host is a second controller fighting the first, which is the exact failure
// the arbiter exists to prevent, moved down a layer where it is harder to see.
//
// WIRING (verified 2026-08-30, see ../README.md)
//   adapter TX -> GPIO 17      adapter RX -> GPIO 16      GND -> GND
//   Getting TX/RX crossed produces a silent bus and no other symptom.
//
// PROTOCOL, newline-terminated ASCII both ways.
//
//   host -> esp32
//     p <deg>     target angle, signed degrees, 0 = centre
//     v <deg/s>   slew speed limit
//     e <0|1>     torque enable / disable (0 lets you move it by hand)
//     c           centre, same as `p 0`
//     ?           force one status line immediately
//     i           identity banner
//
//   esp32 -> host, streamed at STATUS_HZ
//     s <deg> <target> <moving> <load> <volts> <temp> <errs>
//
//   Lines beginning '#' are human-readable and carry no state.
//
// Angles, not counts, cross this wire. The STS3215's 0..4095 encoder is an
// implementation detail of this file; nothing upstream should have to know that
// centre happens to be 2048.

#include <SCServo.h>

// ---- configuration ---------------------------------------------------------

const int      RX_PIN    = 17;          // ESP32 receives on 17 <- adapter TX
const int      TX_PIN    = 16;          // ESP32 transmits on 16 -> adapter RX
const uint32_t BUS_BAUD  = 1000000;     // STS3215 factory default
const uint32_t USB_BAUD  = 115200;
const uint8_t  PAN_ID    = 1;

const int   CENTRE    = 2048;           // encoder counts, 4096 = one turn
const float CPD       = 4096.0f / 360.0f;   // counts per degree
const float LIMIT_DEG = 80.0f;          // hard clamp, both directions

// A hard clamp lives here as well as in pan_node because this is the layer that
// cannot be talked out of it. A bad angle from a ROS bug, a fat-fingered manual
// serial command, or a garbled byte all stop at the same wall, and the wall is
// below the point where the camera ribbon starts to complain.

const uint16_t SLEW_MAX   = 4000;       // counts/s ceiling handed to the servo
const uint8_t  ACCEL      = 50;
const float    STATUS_HZ  = 50.0f;

// ---- state -----------------------------------------------------------------

SMS_STS  st;
float    targetDeg  = 0.0f;
uint16_t slewCounts = 2400;             // ~210 deg/s, brisk but not violent
long     errs       = 0;
bool     torque     = true;
uint32_t lastStatus = 0;
String   line;

// ---- helpers ---------------------------------------------------------------

float clampDeg(float d) {
  if (d >  LIMIT_DEG) return  LIMIT_DEG;
  if (d < -LIMIT_DEG) return -LIMIT_DEG;
  return d;
}

int degToCounts(float deg) { return CENTRE + (int)lroundf(deg * CPD); }
float countsToDeg(int c)   { return (c - CENTRE) / CPD; }

void applyTarget() {
  st.WritePosEx(PAN_ID, degToCounts(targetDeg), slewCounts, ACCEL);
}

void banner() {
  Serial.println("# esp32_pan 1.0  doggobot camera pan axis");
  Serial.printf("# servo id %d, limit +/-%.0f deg, bus %lu baud\n",
                PAN_ID, LIMIT_DEG, (unsigned long)BUS_BAUD);
  Serial.println("# p <deg> | v <deg/s> | e <0|1> | c | ? | i");
}

void status() {
  int pos = st.ReadPos(PAN_ID);
  if (pos < 0) {                        // -1 is the library's read failure
    errs++;
    Serial.printf("s nan %.2f 0 0 0.0 0 %ld\n", targetDeg, errs);
    return;
  }
  int   load  = st.ReadLoad(PAN_ID);
  int   mov   = st.ReadMove(PAN_ID);
  int   volt  = st.ReadVoltage(PAN_ID);
  int   temp  = st.ReadTemper(PAN_ID);
  Serial.printf("s %.2f %.2f %d %d %.1f %d %ld\n",
                countsToDeg(pos), targetDeg, mov < 0 ? 0 : mov,
                load, volt < 0 ? 0.0f : volt / 10.0f,
                temp < 0 ? 0 : temp, errs);
}

// ---- command handling ------------------------------------------------------

void handle(String s) {
  s.trim();
  if (!s.length()) return;
  char c = s.charAt(0);
  String arg = s.substring(1);
  arg.trim();

  switch (c) {
    case 'p':
      targetDeg = clampDeg(arg.toFloat());
      applyTarget();
      break;

    case 'c':
      targetDeg = 0.0f;
      applyTarget();
      break;

    case 'v': {
      float dps = arg.toFloat();
      if (dps > 0) {
        long ct = lroundf(dps * CPD);
        slewCounts = (uint16_t)(ct > SLEW_MAX ? SLEW_MAX : (ct < 50 ? 50 : ct));
        Serial.printf("# slew %u counts/s\n", slewCounts);
      }
      break;
    }

    case 'e':
      torque = (arg.toInt() != 0);
      st.EnableTorque(PAN_ID, torque ? 1 : 0);
      Serial.printf("# torque %s\n", torque ? "on" : "off");
      break;

    case '?':
      status();
      break;

    case 'i':
      banner();
      break;

    default:
      Serial.printf("# ignored: %s\n", s.c_str());
      break;
  }
}

// ---- arduino ---------------------------------------------------------------

void setup() {
  Serial.begin(USB_BAUD);
  Serial1.begin(BUS_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);
  st.pSerial = &Serial1;
  delay(300);

  banner();
  if (st.Ping(PAN_ID) == -1) {
    // Say so and carry on. The host can then report "pan axis missing" instead
    // of silently steering a camera that is not there, and a servo plugged in
    // late still works without a power cycle.
    Serial.printf("# WARNING no response from id %d, check power and wiring\n",
                  PAN_ID);
    errs++;
  } else {
    Serial.printf("# servo %d present\n", PAN_ID);
  }

  st.EnableTorque(PAN_ID, 1);
  applyTarget();
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (line.length()) { handle(line); line = ""; }
    } else if (line.length() < 64) {
      line += ch;
    }
  }

  uint32_t now = millis();
  if (now - lastStatus >= (uint32_t)(1000.0f / STATUS_HZ)) {
    lastStatus = now;
    status();
  }
}
