#include "HapticMotor.h"
#include <ESP32Servo.h>
#include <Arduino.h>

static const int SERVO_NEUTRAL = 90;
static const int SERVO_PULL = 45; // adjust to create a tug

HapticMotor::HapticMotor(uint8_t pin) : _pin(pin) {}

void HapticMotor::begin() {
  servo::attach(_pin);
  servo::write(SERVO_NEUTRAL);
  delay(50);
}

void HapticMotor::tugOnce(unsigned long ms) {
  servo::write(SERVO_PULL);
  delay(ms);
  servo::write(SERVO_NEUTRAL);
}

void HapticMotor::tugStop(unsigned long ms) {
  // a stronger/longer pull to indicate stop
  servo::write(SERVO_PULL - 10);
  delay(ms);
  servo::write(SERVO_NEUTRAL);
}
