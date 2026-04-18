#include "VL53L4CDSensor.h"
#include <Arduino.h>
#ifdef USE_SPARKFUN_VL53L4CD
#include <SparkFun_VL53L4CD.h>
#endif

VL53L4CDSensor::VL53L4CDSensor() {
}

bool VL53L4CDSensor::begin() {
  // Initialize real sensor if library is available
#ifdef USE_SPARKFUN_VL53L4CD
  // The SparkFun library exposes VL53L4CD class; construct on the stack
  static VL53L4CD dev;
  if (!dev.begin()) {
    return false;
  }
  // Start ranging in default mode
  dev.startRanging();
  // store pointer into user-data area if you want multiple instances
  return true;
#else
  // Fallback stub
  return true;
#endif
}

int VL53L4CDSensor::readDistance() {
  // Use real library when available
#ifdef USE_SPARKFUN_VL53L4CD
  // Note: This assumes a single global device instance per multiplexer channel.
  // If using multiple instances, adapt to hold one VL53L4CD object per sensor.
  static VL53L4CD dev;
  if (dev.checkForDataReady()) {
    uint16_t distance = dev.getRange();
    dev.clearInterrupt();
    return (int)distance;
  }
  return -1;
#else
  // Simulation / stub mode
#ifdef SIMULATE_SENSORS
  static int counter = 0;
  counter++;
  float v = 1500.0f + 500.0f * sinf(0.1f * counter);
  return (int)v;
#else
  return -1;
#endif
#endif
}
