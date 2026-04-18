#include "Multiplexer.h"
#include <Wire.h>
#include <Arduino.h>

Multiplexer::Multiplexer(uint8_t i2c_addr) : _addr(i2c_addr) {}

bool Multiplexer::begin() {
  Wire.beginTransmission(_addr);
  return Wire.endTransmission() == 0;
}

void Multiplexer::selectChannel(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(_addr);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delay(5);
}
