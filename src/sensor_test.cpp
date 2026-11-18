// =====================================================================
//  Sensor test - prints every sensor reading to the serial monitor.
//  Motors are held off. Uses the pins and flags from include/config.h
//  (USE_MUX, SENSORS_REVERSED), so what you see here is exactly what
//  the PID firmware sees.
//
//  Upload:   pio run -e sensor_test_old -t upload   (or sensor_test_new)
//  Monitor:  pio device monitor            (115200 baud)
//
//  Columns S1..S8 = left to right as seen from above, raw ADC 0..1023.
//  Commands:  m = show min / max seen so far     r = reset min / max
//             f = faster / slower printing
// =====================================================================
#include <Arduino.h>
#include "config.h"

static uint16_t val[NUM_SENSORS];
static uint16_t vMin[NUM_SENSORS];
static uint16_t vMax[NUM_SENSORS];
static uint16_t periodMs = 200;
static uint16_t lines = 0;

static void readSensors() {
  for (uint8_t ch = 0; ch < NUM_SENSORS; ch++) {
#if USE_MUX
    digitalWrite(PIN_MUX_S0, ch & 1);
    digitalWrite(PIN_MUX_S1, (ch >> 1) & 1);
    digitalWrite(PIN_MUX_S2, (ch >> 2) & 1);
    delayMicroseconds(200);
    analogRead(PIN_MUX_Z);                 // throw-away read after switching
    uint16_t v = analogRead(PIN_MUX_Z);
#else
    analogRead(A0 + ch);
    uint16_t v = analogRead(A0 + ch);
#endif
#if SENSORS_REVERSED
    uint8_t i = NUM_SENSORS - 1 - ch;
#else
    uint8_t i = ch;
#endif
    val[i] = v;
    if (v < vMin[i]) vMin[i] = v;
    if (v > vMax[i]) vMax[i] = v;
  }
}

static void printPadded(uint16_t v) {
  if (v < 1000) Serial.print(' ');
  if (v < 100)  Serial.print(' ');
  if (v < 10)   Serial.print(' ');
  Serial.print(v);
  Serial.print(F("  "));
}

static void printRow(const __FlashStringHelper *label, const uint16_t *a) {
  Serial.print(label);
  for (uint8_t i = 0; i < NUM_SENSORS; i++) printPadded(a[i]);
  Serial.println();
}

static void printHeader() {
  Serial.print(F("       "));
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F("  S")); Serial.print(i + 1); Serial.print(F("  "));
  }
  Serial.println();
}

static void resetMinMax() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) { vMin[i] = 1023; vMax[i] = 0; }
}

void setup() {
  // keep the motors off
  pinMode(PIN_L_FWD, OUTPUT); pinMode(PIN_L_REV, OUTPUT);
  pinMode(PIN_R_FWD, OUTPUT); pinMode(PIN_R_REV, OUTPUT);
  digitalWrite(PIN_L_FWD, LOW); digitalWrite(PIN_L_REV, LOW);
  digitalWrite(PIN_R_FWD, LOW); digitalWrite(PIN_R_REV, LOW);
#if USE_MUX
  pinMode(PIN_MUX_S0, OUTPUT); pinMode(PIN_MUX_S1, OUTPUT); pinMode(PIN_MUX_S2, OUTPUT);
#endif
  pinMode(PIN_LED, OUTPUT);

  Serial.begin(115200);
  resetMinMax();
  Serial.println(F("\nSensor test  (m = min/max, r = reset, f = speed)"));
#if USE_MUX
  Serial.println(F("Mode: 74HC4051 mux on A0"));
#else
  Serial.println(F("Mode: direct A0..A7"));
#endif
}

void loop() {
  static uint32_t last = 0;

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'm') { printHeader(); printRow(F("min:   "), vMin); printRow(F("max:   "), vMax); }
    if (c == 'r') { resetMinMax(); Serial.println(F("min/max reset")); }
    if (c == 'f') { periodMs = (periodMs == 200) ? 50 : 200; }
  }

  readSensors();
  if (millis() - last < periodMs) return;
  last = millis();

  if (lines % 20 == 0) printHeader();
  lines++;
  printRow(F("raw:   "), val);
  digitalWrite(PIN_LED, !digitalRead(PIN_LED));
}
