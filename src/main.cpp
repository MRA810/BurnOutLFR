// =====================================================================
//  Line Follower Robot - PID firmware for the competition track
//  (solid black start / finish boxes, dashed lines, crossings, sharp corners)
//  Arduino Nano + 8x TCRT5000 analog array + L298N
//
//  All settings are in this one file; it does not use config.h.
//
//  ONE FIRMWARE, TWO MODES - only the power switch and the reset button
//
//  STATUS LEDs:  RED = calibration    GREEN = ready    BLUE = racing
//
//  RACE:       switch on, or press reset ONCE.
//      GREEN fast blink 2 s   (press reset again now to calibrate instead)
//      GREEN solid            ready, hands off
//      BLUE solid             racing
//      BLUE slow blink        stopped (finish box, or line lost)
//
//  CALIBRATE:  press reset, then press it AGAIN while GREEN blinks fast.
//      RED solid 3 s          get ready, robot flat on the track
//      RED slow blink 20 s    slide the robot by hand: white -> into the
//                             black box (hold) -> back to white (hold), repeat
//      GREEN solid            saved. Press reset once to race.
//      RED fast blink         failed, old calibration kept. Calibrate again.
//
//  Serial (115200, bench only - unplug USB for the run):
//    s        toggle sensor stream (raw + normalised + error)
//    p<val>   set Kp      i<val>  set Ki      d<val>  set Kd
//    b<val>   set base speed          c<val>  set curve speed
//    x        stop motors             g       run again
//    ?        print current settings
// =====================================================================
#include <Arduino.h>
#include <EEPROM.h>

// ---------------- Pins ------------------------------------------------
#define USE_MUX          0     // 0 = array wired straight to A0..A7
                               // 1 = 74HC4051 (Z->A0, S0->D2, S1->D4, S2->D7)
#define PIN_MUX_Z        A0
#define PIN_MUX_S0       2
#define PIN_MUX_S1       4
#define PIN_MUX_S2       7

#define PIN_L_FWD        11    // L298N IN1  (left motor)
#define PIN_L_REV        10    // L298N IN2
#define PIN_R_FWD        9     // L298N IN3  (right motor)
#define PIN_R_REV        6     // L298N IN4
#define PIN_LED_RED      4     // calibration mode
#define PIN_LED_GREEN    2     // ready mode
#define PIN_LED_BLUE     13    // racing mode (also lights the Nano's own LED)

#if USE_MUX
#error "D2 and D4 are used by the status LEDs, so the multiplexer cannot be used"
#endif

// ---------------- Direction fixes (no rewiring needed) ----------------
#define LEFT_MOTOR_INVERT   0  // 1 if the left wheel spins backwards
#define RIGHT_MOTOR_INVERT  0  // 1 if the right wheel spins backwards
#define SWAP_MOTORS         0  // 1 if left/right motors are exchanged
#define SENSORS_REVERSED    0  // 1 if index 0 is the RIGHT-most sensor
#define SENSOR_DISABLE_MASK 0x00  // ignore faulty sensors: bit0 = S1 ... bit7 = S8
#define LINE_READS_HIGH     1  // 1 = line gives the HIGHER ADC value

// ---------------- Inverse colours -------------------------------------
#define AUTO_INVERT         0       // 1 = switch automatically between a black
                                    //   line on white and a white line on black
#define POLARITY_SAMPLES    6       // consecutive readings needed to switch
#define LIGHT_LEVEL         250     // 0..1000, below this a sensor counts as
                                    //   clearly "off the line"

// ---------------- Geometry --------------------------------------------
// ---------------- Robot and track dimensions -------------------------
#define NUM_SENSORS         8
#define SENSOR_PITCH_MM     15.0f   // sensor to sensor
#define SENSOR_LEVER_MM     147.0f  // sensor row ahead of the wheel axle
#define GAP_LENGTH_MM       100.0f  // longest gap in the line
// Robot speed at GAP_SPEED, in mm per second. Measure it: time the robot
// over 1 m of straight line and put 1000 / seconds here. The gap and
// corner timings below are calculated from it.
#define EST_SPEED_MM_S      600.0f
// Half span of the array = 3.5 * 15 = 52.5 mm  -> error range +-52.5 mm

// ---------------- PID (error is in millimetres) -----------------------
#define KP                  2.0f    // PWM per mm of error
#define KI                  0.0f    // PWM per mm*s      (usually 0)
#define KD                  0.12f   // PWM per mm/s
#define I_LIMIT             60.0f   // anti wind-up (PWM)
#define D_FILTER            0.10f   // 0..1, lower = smoother derivative

// ---------------- Speeds (PWM 0..255) ---------------------------------
#define BASE_SPEED          140     // normal following speed
#define STRAIGHT_SPEED      190     // top speed on straights (set equal to
                                    //   BASE_SPEED to switch the boost off)
#define STRAIGHT_ERR_MM     8.0f    // line within this of centre = "straight"
#define STRAIGHT_AFTER_MS   120     // straight for this long before speeding up
#define CURVE_SPEED         100     // speed when the line is at the array edge
#define MAX_SPEED           255
#define MAX_REVERSE         150     // inner wheel may reverse on tight curves
#define TURN_SPEED          100     // spin speed for sharp turns
#define PIVOT_ERR_MM        45.0f   // line this far from centre -> sharp turn:
                                    //   one wheel forward, the other backward
#define PIVOT_EXIT_MM       25.0f   // keep spinning until the line is back
                                    //   within this distance of centre
#define GAP_SPEED           120     // straight speed while crossing a gap

// ---------------- Line-lost behaviour ---------------------------------
#define ON_LINE_LEVEL       350     // 0..1000, sensor counts as "on the line"
#define NOISE_FLOOR         120     // 0..1000, below this is ignored
#define GROUP_LEVEL         250     // 0..1000, sensors above this that sit side
                                    //   by side are treated as one line
#define WIDE_GROUP          5       // this many sensors side by side = crossing,
                                    //   junction or box: hold course, don't steer
#define T_TURN_DIR          -1      // which way to try first: -1 = left, 1 = right.
                                    //   Used at a T junction (line ends on a cross
                                    //   bar), at a fork, and when searching for a
                                    //   lost line
#define FORK_BIAS_MM        10.0f   // at a fork, how strongly to favour the branch
                                    //   on the T_TURN_DIR side
#define GAP_CENTER_MM       16.0f   // line lost while within this of centre
                                    //   -> treat as a gap and go straight
#define EDGE_MEMORY_MS      300     // how long a side branch (L corner) is remembered
#define CORNER_ROLL         0.6f    // at an L corner roll forward this fraction
                                    //   of the sensor-to-axle distance, then spin.
                                    //   Raise it if the robot cuts the corner,
                                    //   lower it if it overshoots
#define LOST_STOP_MS        2500    // nothing found for this long -> stop

#define STOP_ON_FULL_BAR    1       // 1 = stop in the solid black finish box
#define FINISH_DEPTH_MM     80.0f   // solid black for this distance = finish box
                                    //   (a crossing line is only 30 mm deep)
#define BOX_EXIT_MS         2500    // max time to drive out of the start box

// ---------------- Start-up sequence -----------------------------------
#define RESET_WINDOW_MS     2000    // press reset again within this time after a
                                    //   reset to enter calibration mode
#define HAND_CAL_SECONDS    20      // length of the hand calibration
#define HAND_CAL_MIN_RANGE  120     // smallest white-to-black swing accepted
#define MODE_FLAG_ADDR      100     // EEPROM byte used to detect the double reset
#define MODE_FLAG_VALUE     0xA5
#define CALIBRATE_AT_START  0       // 0 = always use the calibration saved by
                                    //     calibrate.cpp (slow hand calibration)
                                    // 1 = when not in the start box, calibrate by
                                    //     turning over the line at power-on
#define START_DELAY_MS      500    // hands-off time after power on / reset
#define CAL_SPEED           150     // pivot speed during calibration
#define CAL_TIMEOUT_MS      2500    // max time for each calibration turn
#define CAL_OVERRUN_MS      160     // keep turning this long after the edge
                                    //   sensor reaches the line
#define CAL_SEEN_RANGE      120     // ADC swing that counts as "saw the line"
#define CAL_MIN_RANGE       60      // min (max-min) ADC counts per sensor
#define GO_DELAY_MS         1000    // pause after calibration, then go

#define LOOP_US             1000    // control period (1000 Hz)

// ----------------------------- state ---------------------------------
enum State : uint8_t { ST_RUN, ST_STOPPED, ST_CAL_FAIL, ST_CAL_DONE };
static State state = ST_STOPPED;

static uint16_t rawv[NUM_SENSORS];
static uint16_t calMin[NUM_SENSORS];
static uint16_t calMax[NUM_SENSORS];
static uint16_t calScale[NUM_SENSORS];  // 64000 / (max - min), set after calibration
static uint16_t norm[NUM_SENSORS];      // 0..1000, 1000 = on the line

static float kp = KP, ki = KI, kd = KD;
static int16_t baseSpeed = BASE_SPEED, curveSpeed = CURVE_SPEED;

static float errMm = 0, prevErr = 0, integ = 0, dFilt = 0;
static uint8_t onCount = 0;
static bool streamOn = false;
static uint8_t lineWidth = 0;        // sensors in the line group being followed
static uint8_t grpSt = 0, grpEnd = 0; // first sensor of that group, and one past its last
static bool leavingBox = false;      // true while driving out of the start box
static bool inverted = false;        // true while following a white line on black

static const float HALF_SPAN_MM = (NUM_SENSORS - 1) * 0.5f * SENSOR_PITCH_MM;

// ---- timings calculated from the dimensions above ----
// cross the longest gap, with 50 % margin
static const uint16_t GAP_MS = (uint16_t)(GAP_LENGTH_MM * 1.5f / EST_SPEED_MM_S * 1000.0f);
// forward roll at an L corner before spinning
static const uint16_t CORNER_FWD_MS =
    (uint16_t)(SENSOR_LEVER_MM * CORNER_ROLL / EST_SPEED_MM_S * 1000.0f);
// solid black for this long = finish box
static const uint16_t FULL_BAR_MS = (uint16_t)(FINISH_DEPTH_MM / EST_SPEED_MM_S * 1000.0f);

// ----------------------------- motors --------------------------------
static void driveOne(uint8_t pinF, uint8_t pinR, int16_t v, bool invert) {
  if (invert) v = -v;
  v = constrain(v, -255, 255);
  if (v >= 0) { analogWrite(pinR, 0); analogWrite(pinF, v); }
  else        { analogWrite(pinF, 0); analogWrite(pinR, -v); }
}

// left / right : -255 .. +255, positive = forward
static void setMotors(int16_t left, int16_t right) {
#if SWAP_MOTORS
  int16_t t = left; left = right; right = t;
#endif
  driveOne(PIN_L_FWD, PIN_L_REV, left,  LEFT_MOTOR_INVERT);
  driveOne(PIN_R_FWD, PIN_R_REV, right, RIGHT_MOTOR_INVERT);
}

static void stopMotors() { setMotors(0, 0); }

// ----------------------------- status LEDs ---------------------------
static void leds(bool red, bool green, bool blue) {
  digitalWrite(PIN_LED_RED,   red);
  digitalWrite(PIN_LED_GREEN, green);
  digitalWrite(PIN_LED_BLUE,  blue);
}
static bool blinkSlow() { return (millis() >> 8) & 1; }   // about 2 per second
static bool blinkFast() { return (millis() >> 6) & 1; }   // about 8 per second

// ----------------------------- sensors -------------------------------
static void readRaw() {
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
#if USE_MUX
    digitalWrite(PIN_MUX_S0, ch & 1);
    digitalWrite(PIN_MUX_S1, (ch >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (ch >> 2) & 1);
    delayMicroseconds(30);                 // mux + sensor output settling
    uint16_t v = analogRead(PIN_MUX_Z);
#else
    uint16_t v = analogRead(A0 + ch);
#endif
#if SENSORS_REVERSED
    rawv[NUM_SENSORS - 1 - ch] = v;
#else
    rawv[ch] = v;
#endif
  }
}

// Reads the array, fills norm[] and onCount.
// Returns true and updates errMm when the line is visible.
// errMm: negative = line is left of centre, positive = right (mm).
static bool readLine() {
  readRaw();

  uint8_t lo = 0, hi = NUM_SENSORS - 1;            // outermost enabled sensors
  while (lo < hi && (SENSOR_DISABLE_MASK & (1 << lo))) lo++;
  while (hi > lo && (SENSOR_DISABLE_MASK & (1 << hi))) hi--;

  // normalise every sensor to 0..1000 (1000 = on the line, current colours)
  uint8_t dark = 0, light = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (SENSOR_DISABLE_MASK & (1 << i)) { norm[i] = 0; continue; }
    // one multiply and a shift per sensor (no division in the control loop)
    uint16_t mn = calMin[i];
    int16_t v = 0;
    if (rawv[i] > mn) {
      v = (int16_t)(((uint32_t)(rawv[i] - mn) * calScale[i]) >> 6);
      if (v > 1000) v = 1000;
    }
#if !LINE_READS_HIGH
    v = 1000 - v;
#endif
    if (inverted) v = 1000 - v;
    norm[i] = (uint16_t)v;
    if (v >= ON_LINE_LEVEL) dark++;
    else if (v < LIGHT_LEVEL) light++;
  }

#if AUTO_INVERT
  // Inverted track: both ends of the array are "on the line" and only a
  // narrow strip (1 to 3 sensors) is off it. A corner has one end off the
  // line and a crossing has no sensor off it, so neither triggers this.
  static uint8_t polCount = 0;
  if (norm[lo] >= ON_LINE_LEVEL && norm[hi] >= ON_LINE_LEVEL &&
      light >= 1 && light <= 3 && dark >= 4) {
    if (++polCount >= POLARITY_SAMPLES) {
      polCount = 0;
      inverted = !inverted;
      for (uint8_t i = 0; i < NUM_SENSORS; i++)
        if (!(SENSOR_DISABLE_MASK & (1 << i))) norm[i] = 1000 - norm[i];
    }
  } else {
    polCount = 0;
  }
#endif

  onCount = 0;
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    if (norm[i] >= ON_LINE_LEVEL) onCount++;
  if (onCount == 0) return false;

  // Split the array into groups of neighbouring sensors that see black and
  // follow the group nearest to where the line was last. A crossing line or
  // a second branch is then a separate group and does not pull the steering.
  float bestC = 0, bestDist = 1e9f;
  uint8_t bestW = 0;
  bool found = false;
  uint8_t i = 0;
  while (i < NUM_SENSORS) {
    if (norm[i] < GROUP_LEVEL) { i++; continue; }
    uint8_t st = i;
    long ws = 0, sm = 0;
    bool strong = false;
    while (i < NUM_SENSORS && norm[i] >= GROUP_LEVEL) {
      uint16_t w = norm[i] - NOISE_FLOOR;
      ws += (long)w * (2 * i - (NUM_SENSORS - 1));       // -7,-5 .. +5,+7
      sm += w;
      if (norm[i] >= ON_LINE_LEVEL) strong = true;
      i++;
    }
    if (!strong) continue;
    // the faint sensor on each side of the group sharpens the position
    if (st > 0 && norm[st - 1] > NOISE_FLOOR) {
      uint16_t w = norm[st - 1] - NOISE_FLOOR;
      ws += (long)w * (2 * (st - 1) - (NUM_SENSORS - 1)); sm += w;
    }
    if (i < NUM_SENSORS && norm[i] > NOISE_FLOOR) {
      uint16_t w = norm[i] - NOISE_FLOOR;
      ws += (long)w * (2 * i - (NUM_SENSORS - 1)); sm += w;
    }
    float c = (float)ws / (float)sm * (SENSOR_PITCH_MM * 0.5f);
    float dist = fabs(c - errMm);
    if ((c - errMm) * T_TURN_DIR > 0) dist -= FORK_BIAS_MM;   // favour the preferred side
    if (dist < bestDist) { bestDist = dist; bestC = c; bestW = i - st; grpSt = st; grpEnd = i; found = true; }
  }
  if (!found) return false;
  lineWidth = bestW;
  if (bestW < WIDE_GROUP) errMm = bestC;   // wide block: keep the current course
  return true;
}

// ----------------------------- calibration ---------------------------
#define CAL_MAGIC 0xC47B
struct SavedCal { uint16_t magic; uint16_t mn[NUM_SENSORS]; uint16_t mx[NUM_SENSORS]; };

static void computeScale() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t range = (calMax[i] > calMin[i]) ? (calMax[i] - calMin[i]) : 0;
    calScale[i] = range ? (uint16_t)(64000UL / range) : 0;
  }
}

static void saveCal() {
  SavedCal c;
  c.magic = CAL_MAGIC;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { c.mn[i] = calMin[i]; c.mx[i] = calMax[i]; }
  EEPROM.put(0, c);
}

static bool loadCal() {
  SavedCal c;
  EEPROM.get(0, c);
  if (c.magic != CAL_MAGIC) return false;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { calMin[i] = c.mn[i]; calMax[i] = c.mx[i]; }
  computeScale();
  return true;
}

static void calSample() {
  readRaw();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (rawv[i] < calMin[i]) calMin[i] = rawv[i];
    if (rawv[i] > calMax[i]) calMax[i] = rawv[i];
  }
}

static bool calSeen(uint8_t i) { return calMax[i] >= calMin[i] + CAL_SEEN_RANGE; }

// Pivot in one direction until an outermost sensor (one that has not yet
// seen the line) has crossed it, or until the timeout.
static void calSweepUntilEdge(int16_t dir, uint8_t lo, uint8_t hi) {
  bool loDone = calSeen(lo), hiDone = calSeen(hi);
  setMotors(dir * CAL_SPEED, -dir * CAL_SPEED);
  uint32_t t0 = millis();
  while (millis() - t0 < CAL_TIMEOUT_MS) {
    calSample();
    leds(blinkSlow(), 0, 0);
    if ((!loDone && calSeen(lo)) || (!hiDone && calSeen(hi))) break;
  }
  // carry on briefly so the sensor passes fully over the line
  t0 = millis();
  while (millis() - t0 < CAL_OVERRUN_MS) calSample();
  stopMotors();
  delay(80);
}

// Pivot one way until one edge sensor has crossed the line, then the other
// way until the opposite edge sensor has crossed it, then turn back until
// the line is centred. Sensor-driven, so unequal motors do not matter.
// Returns false if any enabled sensor did not see both line and background.
static bool calibrate() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { calMin[i] = 1023; calMax[i] = 0; }

  uint8_t lo = 0, hi = NUM_SENSORS - 1;            // outermost enabled sensors
  while (lo < hi && (SENSOR_DISABLE_MASK & (1 << lo))) lo++;
  while (hi > lo && (SENSOR_DISABLE_MASK & (1 << hi))) hi--;

  calSample();
  calSweepUntilEdge(+1, lo, hi);
  calSweepUntilEdge(-1, lo, hi);
  stopMotors();

  bool ok = true;
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    if (!(SENSOR_DISABLE_MASK & (1 << i)) &&
        calMax[i] < calMin[i] + CAL_MIN_RANGE) ok = false;

  Serial.print(F("cal min:"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(calMin[i]); }
  Serial.print(F("\ncal max:"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(calMax[i]); }
  Serial.println(ok ? F("\ncal OK") : F("\ncal FAILED - see which sensor has min close to max"));
  if (!ok) return false;

  // pre-compute the scaling so the control loop needs no division
  computeScale();

  // re-centre on the line (max 1.5 s)
  uint32_t t0 = millis();
  errMm = 0;
  while (millis() - t0 < 1500) {
    bool seen = readLine();
    if (seen && fabs(errMm) < 6.0f) break;
    int16_t dir = (errMm >= 0) ? 1 : -1;
    setMotors(dir * (CAL_SPEED - 15), -dir * (CAL_SPEED - 15));
  }
  stopMotors();
  return true;
}

// Slow calibration by hand, motors off. Each sample is the average of four
// readings so a single noisy value or flash of glare cannot set a limit.
// Saves to EEPROM only if every enabled sensor saw a clear swing.
static bool handCalibrate() {
  stopMotors();
  uint16_t mn[NUM_SENSORS], mx[NUM_SENSORS];
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { mn[i] = 1023; mx[i] = 0; }

  Serial.println(F("\n=== CALIBRATION MODE ==="));
  Serial.println(F("3 s: put the robot flat on the track, sensors over white"));
  leds(1, 0, 0);                                   // RED solid: get ready
  delay(3000);
  Serial.println(F("GO: slide white -> black box (hold) -> white (hold), repeat"));

  uint32_t t0 = millis(), lastPrint = 0;
  while (millis() - t0 < HAND_CAL_SECONDS * 1000UL) {
    uint16_t sum[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) sum[i] = 0;
    for (uint8_t k = 0; k < 4; k++) {
      readRaw();
      for (uint8_t i = 0; i < NUM_SENSORS; i++) sum[i] += rawv[i];
    }
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      uint16_t v = sum[i] >> 2;
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
    }
    leds(blinkSlow(), 0, 0);                       // RED slow blink: slide the robot
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      Serial.print(HAND_CAL_SECONDS - (millis() - t0) / 1000); Serial.print(F(" s  swing:"));
      for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(mx[i] - mn[i]); }
      Serial.println();
    }
  }

  bool ok = true;
  Serial.print(F("min:  ")); for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(mn[i]); }
  Serial.print(F("\nmax:  ")); for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(' '); Serial.print(mx[i]); }
  Serial.print(F("\nswing:"));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t r = mx[i] - mn[i];
    Serial.print(' '); Serial.print(r);
    if (!(SENSOR_DISABLE_MASK & (1 << i)) && r < HAND_CAL_MIN_RANGE) { ok = false; Serial.print(F("(FAIL)")); }
  }
  Serial.println();

  if (!ok) {
    Serial.println(F("NOT SAVED - old calibration kept. Double-reset to try again."));
    loadCal();
    return false;
  }
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { calMin[i] = mn[i]; calMax[i] = mx[i]; }
  computeScale();
  saveCal();
  Serial.println(F("SAVED. Press reset once to run. (send s for live values)"));
  return true;
}

// ----------------------------- helpers -------------------------------
static void resetPid() { prevErr = errMm; integ = 0; dFilt = 0; }

static void printSettings() {
  Serial.print(F("Kp=")); Serial.print(kp, 3);
  Serial.print(F(" Ki=")); Serial.print(ki, 3);
  Serial.print(F(" Kd=")); Serial.print(kd, 3);
  Serial.print(F(" base=")); Serial.print(baseSpeed);
  Serial.print(F(" curve=")); Serial.print(curveSpeed);
  Serial.print(F(" gap_ms=")); Serial.print(GAP_MS);
  Serial.print(F(" corner_ms=")); Serial.println(CORNER_FWD_MS);
}

static void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 's': streamOn = !streamOn; break;
    case 'p': kp = Serial.parseFloat(); printSettings(); break;
    case 'i': ki = Serial.parseFloat(); integ = 0; printSettings(); break;
    case 'd': kd = Serial.parseFloat(); printSettings(); break;
    case 'b': baseSpeed  = constrain(Serial.parseInt(), 0, 255); printSettings(); break;
    case 'c': curveSpeed = constrain(Serial.parseInt(), 0, 255); printSettings(); break;
    case 'x': stopMotors(); state = ST_STOPPED; break;
    case 'g': if (state == ST_STOPPED) { resetPid(); state = ST_RUN; } break;
    case '?': printSettings(); break;
    default: break;
  }
}

static void printStream() {
  static uint32_t last = 0;
  if (!streamOn || millis() - last < 100) return;
  last = millis();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(rawv[i]); Serial.print(' '); }
  Serial.print(F("| "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(norm[i]); Serial.print(' '); }
  Serial.print(F("| err(mm)=")); Serial.print(errMm, 1);
  Serial.print(F(" heading(deg)=")); Serial.print(atan2(errMm, SENSOR_LEVER_MM) * 57.2958f, 1);
  Serial.print(F(" on=")); Serial.print(onCount);
  Serial.print(F(" width=")); Serial.print(lineWidth);
  Serial.println(inverted ? F(" INVERTED") : F(""));
}

// ----------------------------- setup ---------------------------------
void setup() {
  pinMode(PIN_LED_RED, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT); pinMode(PIN_LED_BLUE, OUTPUT);
  leds(0, 0, 0);
  pinMode(PIN_L_FWD, OUTPUT); pinMode(PIN_L_REV, OUTPUT);
  pinMode(PIN_R_FWD, OUTPUT); pinMode(PIN_R_REV, OUTPUT);
  stopMotors();
#if USE_MUX
  pinMode(PIN_MUX_S0, OUTPUT); pinMode(PIN_MUX_S1, OUTPUT); pinMode(PIN_MUX_S2, OUTPUT);
#endif

  // ADC prescaler 32 -> ~26 us per conversion instead of 104 us
  ADCSRA = (ADCSRA & ~0x07) | 0x05;

  Serial.begin(115200);
  Serial.setTimeout(40);
  Serial.println(F("\nLFR PID firmware (track version)"));

  // ---- double reset = calibration mode ----
  // A flag is set for the first 2 s after every start. If reset is pressed
  // again inside that time, the next start finds the flag still set.
  if (EEPROM.read(MODE_FLAG_ADDR) == MODE_FLAG_VALUE) {
    EEPROM.update(MODE_FLAG_ADDR, 0);
    state = handCalibrate() ? ST_CAL_DONE : ST_CAL_FAIL;
    return;
  }
  EEPROM.update(MODE_FLAG_ADDR, MODE_FLAG_VALUE);
  {
    uint32_t t0 = millis();
    while (millis() - t0 < RESET_WINDOW_MS) leds(0, blinkFast(), 0);   // GREEN fast blink
  }
  EEPROM.update(MODE_FLAG_ADDR, 0);

  leds(0, 1, 0);                                   // GREEN solid: ready
  delay(START_DELAY_MS);

  inverted = false;                   // the track is a black line on white

  // In the black start box? (needs a calibration saved from an earlier run)
  bool haveSaved = loadCal();
  bool inBox = false;
  if (haveSaved) {
    uint8_t votes = 0;
    for (uint8_t k = 0; k < 10; k++) {
      readLine();
      if (onCount >= NUM_SENSORS - 1) votes++;
      delay(5);
    }
    inBox = votes >= 8;
  }

  if (inBox) {
    Serial.println(F("start box: using saved calibration"));
    leavingBox = true;
  } else if (!CALIBRATE_AT_START && haveSaved) {
    Serial.println(F("using saved calibration"));
  } else if (!CALIBRATE_AT_START) {
    Serial.println(F("no saved calibration - run calibrate.cpp first"));
    state = ST_CAL_FAIL;
    return;
  } else if (calibrate()) {
    saveCal();
    Serial.println(F("calibration saved"));
  } else if (haveSaved) {
    loadCal();                        // new calibration failed: fall back
    Serial.println(F("using saved calibration"));
    for (uint8_t k = 0; k < 3; k++) { leds(1, 0, 0); delay(120); leds(0, 1, 0); delay(120); }
  } else {
    state = ST_CAL_FAIL;
    return;
  }

  leds(0, 1, 0);                                   // GREEN solid: hands off
  delay(GO_DELAY_MS);
  leds(0, 0, 1);                                   // BLUE solid: racing

  readLine();
  resetPid();
  state = ST_RUN;
  printSettings();
}

// ----------------------------- control -------------------------------
static void controlStep(float dt) {
  static uint32_t lostSince = 0;     // 0 = line currently visible
  static bool     gapMode   = false;
  static bool     cornerMode = false;
  static int8_t   cornerSide = 0;
  static uint32_t edgeMs     = 0;    // when one end sensor last saw the line
  static int8_t   edgeSide   = 0;    // which end: -1 left, +1 right
  static bool     pivoting   = false;
  static uint32_t barMs      = 0;    // when a full-width bar was last seen
  static uint32_t boxStart   = 0;    // when the drive out of the start box began
  static uint16_t narrowMs   = 0;    // time spent on a normal line since start
  static bool     finishArmed = false;
  static uint32_t lastStep   = 0;
  static uint32_t straightSince = 0; // when the line became centred
  static float    boost      = 0;    // extra speed on straights
#if STOP_ON_FULL_BAR
  static uint32_t barSince  = 0;
#endif

  bool seen = readLine();
  uint32_t now = millis();

  // ---------------- drive out of the black start box ----------------
  if (leavingBox) {
    if (boxStart == 0) boxStart = now ? now : 1;
    bool onNormalLine = seen && onCount >= 1 && onCount <= 4;
    if (onNormalLine || now - boxStart >= BOX_EXIT_MS) {
      leavingBox = false;
      errMm = 0;
      readLine();
      resetPid();
    } else {
      setMotors(GAP_SPEED, GAP_SPEED);
      return;
    }
  }

  // the finish stop only becomes active after 0.3 s on a normal line,
  // so the start box itself can never trigger it
  if (now != lastStep) {
    lastStep = now;
    if (seen && onCount <= 4 && narrowMs < 1000) narrowMs++;
  }
  if (narrowMs >= 300) finishArmed = true;

#if STOP_ON_FULL_BAR
  if (finishArmed && seen && onCount >= NUM_SENSORS - 1) {
    if (barSince == 0) barSince = now ? now : 1;
    else if (now - barSince >= FULL_BAR_MS) { stopMotors(); state = ST_STOPPED; return; }
  } else barSince = 0;
#endif
  if (seen && onCount >= NUM_SENSORS - 1) barMs = now ? now : 1;

  if (!seen) {
    // ---------------- line lost ----------------
    if (lostSince == 0) {
      lostSince = now ? now : 1;
      cornerMode = (edgeMs != 0) && (now - edgeMs < EDGE_MEMORY_MS);
      cornerSide = edgeSide;
      if (!cornerMode && barMs != 0 && now - barMs < EDGE_MEMORY_MS) {
        cornerMode = true;                      // line ended on a cross bar: T junction
        cornerSide = T_TURN_DIR;
      }
      gapMode = !cornerMode && !pivoting && fabs(errMm) <= GAP_CENTER_MM;   // lost near centre = gap
    }
    uint32_t lostFor = now - lostSince;

    if (lostFor >= LOST_STOP_MS) { stopMotors(); state = ST_STOPPED; return; }

    if (cornerMode) {
      if (lostFor < CORNER_FWD_MS) {
        setMotors(GAP_SPEED, GAP_SPEED);        // roll on so the wheels reach the corner
      } else {
        setMotors(cornerSide * TURN_SPEED, -cornerSide * TURN_SPEED);
        errMm = cornerSide * HALF_SPAN_MM;      // keep spinning this way once the line shows
        pivoting = true;
      }
    } else if (gapMode && lostFor < GAP_MS) {
      setMotors(GAP_SPEED, GAP_SPEED);          // coast straight over the gap
    } else {
      // sharp / acute turn: pivot toward the side the line was last seen;
      // if it was dead centre, try the preferred side first
      int16_t dir = (fabs(errMm) < 3.0f) ? T_TURN_DIR : ((errMm >= 0) ? 1 : -1);
      errMm = dir * HALF_SPAN_MM;               // keep to this direction
      setMotors(dir * TURN_SPEED, -dir * TURN_SPEED);
    }
    integ = 0;
    straightSince = 0; boost = 0;
    return;
  }

  // ---------------- line visible: PID ----------------
  if (lostSince != 0) { lostSince = 0; gapMode = false; cornerMode = false; resetPid(); }

  // remember a side branch: one end sensor on the line, the other not
  {
    uint8_t lo = 0, hi = NUM_SENSORS - 1;
    while (lo < hi && (SENSOR_DISABLE_MASK & (1 << lo))) lo++;
    while (hi > lo && (SENSOR_DISABLE_MASK & (1 << hi))) hi--;
    // an end sensor only counts as a side branch when it is not simply the
    // followed line itself running near that end (as it does on a curve)
    bool lOn = norm[lo] >= ON_LINE_LEVEL && (lo < grpSt || lo >= grpEnd || lineWidth >= 4);
    bool rOn = norm[hi] >= ON_LINE_LEVEL && (hi < grpSt || hi >= grpEnd || lineWidth >= 4);
    if (lOn != rOn) { edgeSide = rOn ? 1 : -1; edgeMs = now ? now : 1; }
    else if (lOn && rOn) { edgeMs = 0; }        // both ends = crossing: go straight
  }

  float err = errMm;

  // ---------------- sharp turn: spin on the spot ----------------
  if (pivoting) {
    if (fabs(err) < PIVOT_EXIT_MM) { pivoting = false; resetPid(); }
  } else if (fabs(err) > PIVOT_ERR_MM && onCount <= 3) {
    pivoting = true;                            // (a wide blob is a corner, handled above)
  }
  if (pivoting) {
    int16_t dir = (err >= 0) ? 1 : -1;          // line on the right -> spin right
    setMotors(dir * TURN_SPEED, -dir * TURN_SPEED);
    integ = 0;
    straightSince = 0; boost = 0;
    return;
  }

  // ---------------- straights: build up to top speed ----------------
  if (fabs(err) < STRAIGHT_ERR_MM) {
    if (straightSince == 0) straightSince = now ? now : 1;
    if (now - straightSince >= STRAIGHT_AFTER_MS && boost < (float)(STRAIGHT_SPEED - baseSpeed))
      boost += 0.5f;                            // ramps up over roughly 0.15 s
  } else {
    straightSince = 0;
    boost = 0;                                  // line moving away: back to normal speed
  }
  if (boost < 0) boost = 0;
  float d = (err - prevErr) / dt;
  dFilt += D_FILTER * (d - dFilt);
  prevErr = err;

  integ += ki * err * dt;
  integ = constrain(integ, -I_LIMIT, I_LIMIT);

  float corr = kp * err + integ + kd * dFilt;

  // slow down in proportion to how far the line is from centre
  float a = fabs(err) / HALF_SPAN_MM;
  if (a > 1.0f) a = 1.0f;
  float base = baseSpeed + boost - (baseSpeed - curveSpeed) * a;

  float l = base + corr;      // line to the right (err > 0) -> left wheel faster
  float r = base - corr;

  // if one side saturates, take the excess off the other to keep the turn rate
  if (l > MAX_SPEED) { r -= (l - MAX_SPEED); l = MAX_SPEED; }
  if (r > MAX_SPEED) { l -= (r - MAX_SPEED); r = MAX_SPEED; }
  l = constrain(l, -(float)MAX_REVERSE, (float)MAX_SPEED);
  r = constrain(r, -(float)MAX_REVERSE, (float)MAX_SPEED);

  setMotors((int16_t)l, (int16_t)r);
}

// ----------------------------- loop ----------------------------------
void loop() {
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  if (nowUs - lastUs < LOOP_US) return;
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  handleSerial();

  switch (state) {
    case ST_RUN:
      controlStep(dt);
      if (state == ST_RUN) leds(0, 0, 1);          // BLUE solid: racing
      break;

    case ST_STOPPED: {
      stopMotors();
      readLine();                                  // keeps the 's' stream live
      leds(0, 0, blinkSlow());                     // BLUE slow blink: stopped
      break;
    }

    case ST_CAL_FAIL:
      stopMotors();
      readRaw();
      leds(blinkFast(), 0, 0);                     // RED fast blink: calibration failed
      break;

    case ST_CAL_DONE:                              // calibration saved: wait for reset
      stopMotors();
      readLine();                                  // keeps the 's' stream live
      leds(0, 1, 0);                               // GREEN solid: saved, press reset
      break;
  }
  printStream();
}