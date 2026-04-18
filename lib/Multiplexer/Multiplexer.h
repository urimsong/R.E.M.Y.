#pragma once
#include <stdint.h>

class Multiplexer {
public:
  explicit Multiplexer(uint8_t i2c_addr = 0x70);
  bool begin();
  void selectChannel(uint8_t channel);
private:
  uint8_t _addr;
};
