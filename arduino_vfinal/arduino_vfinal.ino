// Haptic servo controller for LiDAR-driven backpack belt
// Arduino Mega 2560 version — uses built-in Servo library.

#include <Servo.h>

Servo leftServo;
Servo rightServo;

const int LEFT_PIN  = 2;
const int RIGHT_PIN = 3;

const int REST_ANGLE  = 0;
const int PULSE_ANGLE = 90;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);  // Don't stall a full second on partial input

  leftServo.attach(LEFT_PIN);
  rightServo.attach(RIGHT_PIN);

  delay(100);  // Let PWM stabilize before first write
  leftServo.write(REST_ANGLE);
  rightServo.write(REST_ANGLE);

  Serial.println("READY");
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 3) return;   // need at least "X:N"

  char side  = cmd.charAt(0);
  int  angle = cmd.substring(2).toInt();
  angle = constrain(angle, 0, 180);

  if      (side == 'L') leftServo.write(angle);
  else if (side == 'R') rightServo.write(angle);
  else return;  // unknown side, don't send OK

  Serial.println("OK");
}