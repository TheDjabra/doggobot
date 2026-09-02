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
//     p <deg>     target angle, signed degrees, 0 = straight ahead
//     v <deg/s>   slew speed limit
//     e <0|1>     torque enable / disable (0 lets you move it by hand)
//     o <deg>     fine trim, in degrees, on top of the servo's own zero
//     z           STORE THE CURRENT POSITION AS ZERO, in the servo's EEPROM
//     c           centre, same as `p 0`
//     ?           force one status line immediately
//     i           identity banner
//
// BOOT STATE: torque OFF, no position commanded. The firmware has no idea how
// the horn is clocked relative to the bracket, so commanding an angle at power
// up would drive the axis to wherever the encoder happens to call centre and
// take the camera cable with it. It powers up limp and waits to be told.
//
// ZEROING, and why it is done in the servo rather than in software. The horn
// fits in 14.4 degree steps (25 teeth), so mechanical straight-ahead never
// lands on the encoder's own centre. Carrying that difference as a software
// offset does not work: the encoder covers exactly ONE turn, so a horn clocked
// 128 degrees out leaves only about 52 degrees of travel on one side before the
// count passes 4095 and WRAPS instead of clamping.
//
// `z` fixes it in the servo. CalibrationOfs writes the current position into
// EEPROM as 2048, so forward IS zero from then on and it survives power cycles.
// The full +/-90 then sits at counts 1024..3072: inside one turn, and symmetric.
//
//   esp32 -> host, streamed at STATUS_HZ
//     s <deg> <target> <moving> <load> <volts> <temp> <errs> <torque>
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
// TRAVEL CEILING. The axis may never go beyond 90 degrees either side of
// centre. Operator instruction, 2026-09-01. This is not a tuning value: the
// working clamp below may be tightened, never widened past it, and the
// static_assert makes an attempt to widen it a build failure rather than a
// discovery made by watching the camera cable wind up.
constexpr float ABS_LIMIT_DEG = 90.0f;
constexpr float LIMIT_DEG     = 90.0f;      // working clamp, at the ceiling
static_assert(LIMIT_DEG <= ABS_LIMIT_DEG,
              "pan working clamp exceeds the +/-90 degree travel ceiling");

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
// Mechanical straight-ahead, in servo degrees from encoder centre. Set with `o`
// once the horn is fitted. Every angle on the wire is measured from HERE, which
// is what makes the clamp below meaningful: a limit of +/-80 is only a real
// protection if it is 80 degrees from where the camera actually points forward.
float    offsetDeg  = 0.0f;
uint16_t slewCounts = 2400;             // ~210 deg/s, brisk but not violent
long     errs       = 0;
bool     torque     = true;
uint32_t lastStatus = 0;
String   line;

// ---- helpers ---------------------------------------------------------------

// The encoder covers ONE turn, 0..4095 counts. A position outside that does not
// clamp, it wraps, and on a free-spinning axis a wrap is how a camera cable gets
// wound up. So the reachable travel depends on where mechanical straight-ahead
// sits: with a horn clocked 128 degrees off encoder centre there is only about
// 52 degrees left on the positive side, however wide LIMIT_DEG is set.
constexpr int   COUNT_LO   = 0;
constexpr int   COUNT_HI   = 4095;
constexpr float EDGE_DEG   = 2.0f;      // never sit right on the last count

float reachMax() { return (COUNT_HI - CENTRE) / CPD - offsetDeg - EDGE_DEG; }
float reachMin() { return (COUNT_LO - CENTRE) / CPD - offsetDeg + EDGE_DEG; }

float clampDeg(float d) {
  if (!(d == d)) return 0.0f;           // NaN from a garbled line
  // Whichever is tighter: the configured clamp, the hard ceiling, or what the
  // encoder can physically represent at this offset.
  const float lim = LIMIT_DEG < ABS_LIMIT_DEG ? LIMIT_DEG : ABS_LIMIT_DEG;
  float hi =  lim, lo = -lim;
  if (reachMax() < hi) hi = reachMax();
  if (reachMin() > lo) lo = reachMin();
  if (hi < lo) { hi = 0.0f; lo = 0.0f; }   // pathological offset: refuse to move
  if (d > hi) return hi;
  if (d < lo) return lo;
  return d;
}

int degToCounts(float deg) {
  return CENTRE + (int)lroundf((deg + offsetDeg) * CPD);
}
float countsToDeg(int c) {
  return (c - CENTRE) / CPD - offsetDeg;
}

void applyTarget() {
  // Final guard, in counts. Everything upstream works in degrees and should
  // already be inside the clamp, but a position outside one turn WRAPS on this
  // servo rather than clamping, so the last thing before the wire checks it.
  int c = degToCounts(targetDeg);
  if (c < 0)    c = 0;
  if (c > 4095) c = 4095;
  st.WritePosEx(PAN_ID, c, slewCounts, ACCEL);
}

void banner() {
  Serial.println("# esp32_pan 1.0  doggobot camera pan axis");
  Serial.printf("# servo id %d, limit +/-%.0f deg, bus %lu baud\n",
                PAN_ID, LIMIT_DEG, (unsigned long)BUS_BAUD);
  Serial.printf("# offset %.2f deg, torque %s\n", offsetDeg, torque ? "on" : "off");
  Serial.println("# p <deg> | v <deg/s> | e <0|1> | o <deg> | z | c | ? | i");
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
  Serial.printf("s %.2f %.2f %d %d %.1f %d %ld %d\n",
                countsToDeg(pos), targetDeg, mov < 0 ? 0 : mov,
                load, volt < 0 ? 0.0f : volt / 10.0f,
                temp < 0 ? 0 : temp, errs, torque ? 1 : 0);
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
      if (torque) {
        // Hold where it currently IS, not where targetDeg happens to say.
        // Engaging torque must never itself be a movement command: the whole
        // point of coming up limp is that nobody knows where the axis is yet.
        int pos = st.ReadPos(PAN_ID);
        if (pos >= 0) targetDeg = clampDeg(countsToDeg(pos));
        st.EnableTorque(PAN_ID, 1);
        applyTarget();
      } else {
        st.EnableTorque(PAN_ID, 0);
      }
      Serial.printf("# torque %s at %.2f deg\n", torque ? "on" : "off", targetDeg);
      break;

    case 'z': {
      // Writes EEPROM, so it is a deliberate act and never happens on boot.
      // Torque comes off first: calibrating while the servo is holding a
      // position means fighting its own loop as centre moves underneath it.
      st.EnableTorque(PAN_ID, 0);
      torque = false;
      st.CalibrationOfs(PAN_ID);
      offsetDeg = 0.0f;
      targetDeg = 0.0f;
      delay(60);
      int pos = st.ReadPos(PAN_ID);
      float lo = clampDeg(-1e6f), hi = clampDeg(1e6f);
      Serial.printf("# ZEROED: here is now 0 deg (encoder %d, expect ~%d). "
                    "Travel %.1f .. %+.1f deg. Still limp; send e 1.\n",
                    pos, CENTRE, lo, hi);
      break;
    }

    case 'o': {
      float was = offsetDeg;
      offsetDeg = arg.toFloat();
      float lo = clampDeg(-1e6f), hi = clampDeg(1e6f);
      Serial.printf("# offset %.2f -> %.2f deg\n", was, offsetDeg);
      Serial.printf("# reachable %.1f .. %+.1f deg", lo, hi);
      if (hi < LIMIT_DEG - 0.5f || lo > -LIMIT_DEG + 0.5f) {
        Serial.printf("  (LESS THAN the %.0f deg asked for: the horn is "
                      "clocked %.0f deg off encoder centre, so the encoder "
                      "runs out first)", LIMIT_DEG, offsetDeg);
      }
      Serial.println();
      break;
    }

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

  torque = false;                 // come up limp, see BOOT STATE above
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

  // Deliberately NOT EnableTorque(1) and NOT applyTarget(). Release the axis so
  // the horn can be fitted and turned to straight ahead by hand, then read the
  // angle off the status stream and hand it back with `o`.
  st.EnableTorque(PAN_ID, 0);
  Serial.println("# limp: torque off, no target. Fit the horn, point it "
                 "forward, then send o <deg> and e 1");
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
