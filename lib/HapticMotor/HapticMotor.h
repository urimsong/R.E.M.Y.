#pragma once
#include <stdint.h>

class HapticMotor {
public:
  explicit HapticMotor(uint8_t pin);
  void begin();
  void tugOnce(unsigned long ms = 200);
  void tugStop(unsigned long ms = 250);
private:
  uint8_t _pin;
};
