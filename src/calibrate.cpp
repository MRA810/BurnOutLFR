// =====================================================================
//  Slow hand calibration for the line follower.
//
//  The motors stay OFF. You move the robot by hand for 20 seconds so every
//  sensor sees pure white and pure black. The result is saved in the Nano's
//  memory, and the main firmware (lfr_best.cpp) uses it at every start.
//
//  Upload:   pio run -e calibrate_new -t upload      (or calibrate_old)
//  Monitor:  pio device monitor                      (115200 baud)
//
//  LED (D13):  slow blink = sampling, keep moving the robot
//              solid      = saved, calibration is good
//              fast blink = NOT saved, at least one sensor failed
//
//  Serial commands after it finishes:   c = calibrate again
//
//  These two settings MUST match the main firmware:
// =====================================================================
#include <Arduino.h>
#include <EEPROM.h>

#define SENSORS_REVERSED    0       // same value as in lfr_best.cpp
#define LINE_READS_HIGH     1       // same value as in lfr_best.cpp

#define NUM_SENSORS         8
#define SAMPLE_SECONDS      20      // how long you have to move the robot
#define MIN_RANGE           60      // smallest white-to-black swing accepted
#define GOOD_RANGE          150     // swing that counts as comfortable

#define PIN_LED             13
const uint8_t MOTOR_PINS[4] = {11, 10, 9, 6};

// must match the main firmware
#define CAL_MAGIC 0xC47B
struct SavedCal { uint16_t magic; uint16_t mn[NUM_SENSORS]; uint16_t mx[NUM_SENSORS]; };

static uint16_t val[NUM_SENSORS], calMin[NUM_SENSORS], calMax[NUM_SENSORS];
static bool saved = false;

// each value is the average of 4 readings, so one noisy sample or a flash
// of glare cannot set the minimum or maximum on its own
static void readSensors() {
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
    uint16_t sum = 0;
    for (uint8_t k = 0; k < 4; k++) sum += analogRead(A0 + ch);
#if SENSORS_REVERSED
    val[NUM_SENSORS - 1 - ch] = sum >> 2;
#else
    val[ch] = sum >> 2;
#endif
  }
}

static void pad(uint16_t v) {
  if (v < 1000) Serial.print(' ');
  if (v < 100)  Serial.print(' ');
  if (v < 10)   Serial.print(' ');
  Serial.print(v);
  Serial.print(F("  "));
}

static void row(const __FlashStringHelper *label, const uint16_t *a) {
  Serial.print(label);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) pad(a[i]);
  Serial.println();
}

static void header() {
  Serial.print(F("          "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { Serial.print(F("  S")); Serial.print(i + 1); Serial.print(F("  ")); }
  Serial.println();
}

static void calibrate() {
  saved = false;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { calMin[i] = 1023; calMax[i] = 0; }

  Serial.println(F("\n=== SLOW CALIBRATION ==="));
  Serial.println(F("Starting in 3 s. Hold the robot flat on the track at normal height."));
  digitalWrite(PIN_LED, HIGH);
  delay(3000);
  Serial.println(F("GO: slide the robot slowly sideways across a black line, there and back,"));
  Serial.println(F("so EVERY sensor passes over black and over white. Keep going until it stops."));
  header();

  uint32_t t0 = millis(), lastPrint = 0;
  while (millis() - t0 < SAMPLE_SECONDS * 1000UL) {
    readSensors();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      if (val[i] < calMin[i]) calMin[i] = val[i];
      if (val[i] > calMax[i]) calMax[i] = val[i];
    }
    digitalWrite(PIN_LED, (millis() >> 8) & 1);
    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      uint16_t range[NUM_SENSORS];
      for (uint8_t i = 0; i < NUM_SENSORS; i++) range[i] = calMax[i] - calMin[i];
      Serial.print(SAMPLE_SECONDS - (millis() - t0) / 1000); Serial.print(F(" s left "));
      row(F("swing:"), range);
    }
  }

  // ---- result ----
  Serial.println(F("\n=== RESULT ==="));
  header();
  row(F("min:      "), calMin);
  row(F("max:      "), calMax);
  uint16_t range[NUM_SENSORS];
  bool ok = true;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    range[i] = calMax[i] - calMin[i];
    if (range[i] < MIN_RANGE) ok = false;
  }
  row(F("swing:    "), range);
  Serial.print(F("verdict:  "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++)
    Serial.print(range[i] < MIN_RANGE ? F(" FAIL  ") : (range[i] < GOOD_RANGE ? F(" weak  ") : F("  ok   ")));
  Serial.println();

  if (ok) {
    SavedCal c;
    c.magic = CAL_MAGIC;
    for (uint8_t i = 0; i < NUM_SENSORS; i++) { c.mn[i] = calMin[i]; c.mx[i] = calMax[i]; }
    EEPROM.put(0, c);
    saved = true;
    Serial.println(F("SAVED. Now upload the main firmware (lfr_best.cpp)."));
  } else {
    Serial.println(F("NOT SAVED: a sensor marked FAIL never saw both black and white."));
    Serial.println(F("Send c to try again and make sure that sensor crosses the line."));
  }
  Serial.println(F("Live check below: 0 = white, 1000 = black. Put a line under each sensor."));
}

void setup() {
  for (uint8_t i = 0; i < 4; i++) { pinMode(MOTOR_PINS[i], OUTPUT); digitalWrite(MOTOR_PINS[i], LOW); }
  pinMode(PIN_LED, OUTPUT);
  ADCSRA = (ADCSRA & ~0x07) | 0x05;      // same ADC speed as the main firmware
  Serial.begin(115200);
  calibrate();
}

void loop() {
  if (Serial.available() && Serial.read() == 'c') { calibrate(); return; }

  // live normalised view, so you can confirm every sensor reads the line
  static uint32_t last = 0;
  readSensors();
  if (millis() - last >= 400) {
    last = millis();
    uint16_t n[NUM_SENSORS];
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      long r = (long)calMax[i] - calMin[i];
      long v = r > 0 ? ((long)val[i] - calMin[i]) * 1000L / r : 0;
      v = constrain(v, 0L, 1000L);
#if !LINE_READS_HIGH
      v = 1000 - v;
#endif
      n[i] = (uint16_t)v;
    }
    row(F("live:     "), n);
  }
  if (saved) digitalWrite(PIN_LED, HIGH);
  else       digitalWrite(PIN_LED, (millis() >> 6) & 1);
}