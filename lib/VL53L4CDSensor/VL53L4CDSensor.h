#pragma once
#include <stdint.h>

class VL53L4CDSensor {
public:
  VL53L4CDSensor();
  // Initialize the sensor on the currently selected I2C bus (multiplexer channel)
  bool begin();
  // Return distance in millimeters, or -1 if unavailable
  int readDistance();
private:
  // internal state for an actual library would go here
};
