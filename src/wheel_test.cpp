// Wheel test: runs each wheel on its own, then both together.
// No sensors and no settings file are used.
//
//   1. LEFT wheel forward      2. LEFT wheel backward
//   3. RIGHT wheel forward     4. RIGHT wheel backward
//   5. BOTH wheels forward     (2 s each, 1 s stop in between), then repeats
//
// Wiring: D11 -> IN1, D10 -> IN2 (left motor), D9 -> IN3, D6 -> IN4 (right motor)
// Upload:  pio run -e wheel_test_new -t upload     (or wheel_test_old)
// Monitor: pio device monitor                      (115200 baud)
//
// Lift the robot so the wheels are off the ground before switching on.
#include <Arduino.h>

const uint8_t LEFT_FWD  = 11;   // IN1
const uint8_t LEFT_REV  = 10;   // IN2
const uint8_t RIGHT_FWD = 9;    // IN3
const uint8_t RIGHT_REV = 6;    // IN4 (moved from D3)

const int SPEED = 180;          // 0..255, same power for every step

void leftWheel(int speed) {     // positive = forward, negative = backward
  analogWrite(LEFT_FWD, speed > 0 ? speed : 0);
  analogWrite(LEFT_REV, speed < 0 ? -speed : 0);
}

void rightWheel(int speed) {
  analogWrite(RIGHT_FWD, speed > 0 ? speed : 0);
  analogWrite(RIGHT_REV, speed < 0 ? -speed : 0);
}

void runStep(const char *label, int left, int right) {
  Serial.println(label);
  leftWheel(left);
  rightWheel(right);
  delay(2000);
  leftWheel(0);
  rightWheel(0);
  delay(1000);
}

void setup() {
  pinMode(LEFT_FWD, OUTPUT);  pinMode(LEFT_REV, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_REV, OUTPUT);
  leftWheel(0);
  rightWheel(0);
  Serial.begin(115200);
  Serial.println("Wheel test - starting in 2 s");
  delay(2000);
}

void loop() {
  runStep("1. LEFT wheel FORWARD",   SPEED, 0);
  runStep("2. LEFT wheel BACKWARD", -SPEED, 0);
  runStep("3. RIGHT wheel FORWARD",  0,  SPEED);
  runStep("4. RIGHT wheel BACKWARD", 0, -SPEED);
  runStep("5. BOTH wheels FORWARD",  SPEED, SPEED);
  Serial.println("--- repeating ---");
  delay(1000);
}