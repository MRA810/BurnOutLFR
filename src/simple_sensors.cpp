// Simple sensor reader, NO multiplexer: prints the 8 raw sensor values.
// Wiring: sensor array A0..A7 -> Nano A0..A7 (the board's LFR SENSOR MODULE
//         header does this), array 5V -> 5V, array GND -> GND.
// Upload:  pio run -e simple_new -t upload     (or simple_old)
// Monitor: pio device monitor                  (115200 baud)
#include <Arduino.h>

const uint8_t SENSOR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

void setup() {
  Serial.begin(115200);
}

void loop() {
  for (uint8_t i = 0; i < 8; i++) {
    analogRead(SENSOR_PINS[i]);              // first read after switching pins is discarded
    Serial.print(analogRead(SENSOR_PINS[i])); // 0..1023, higher = darker
    Serial.print('\t');
  }
  Serial.println();
  delay(200);
}