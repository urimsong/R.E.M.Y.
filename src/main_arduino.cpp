#include <Arduino.h>
#include <Wire.h>
#include "Multiplexer.h"
#include "VL53L4CDSensor.h"
#include "HapticMotor.h"

// Hardware pin configuration (adjust pins as needed)
#define SDA_PIN -1 // default (use board defaults)
#define SCL_PIN -1 // default (use board defaults)

// I2C multiplexer address (TCA9548A)
static const uint8_t MUX_ADDR = 0x70;

// Sensor channels on the multiplexer
static const uint8_t SENSOR_CHANNELS[] = {0, 1, 2};
static const size_t SENSOR_COUNT = sizeof(SENSOR_CHANNELS) / sizeof(SENSOR_CHANNELS[0]);

// Motor pins (adjust to your wiring)
static const uint8_t MOTOR_LEFT_PIN = 16;
static const uint8_t MOTOR_RIGHT_PIN = 17;

// Distance thresholds (millimeters)
static const int STOP_THRESHOLD = 600; // if obstacle closer than this -> stop
static const int TURN_THRESHOLD = 1200; // if left/right much closer than front -> turn

Multiplexer mux(MUX_ADDR);
VL53L4CDSensor sensors[SENSOR_COUNT];
HapticMotor leftMotor(MOTOR_LEFT_PIN);
HapticMotor rightMotor(MOTOR_RIGHT_PIN);

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("KimCheese ESP32-S3 - starting");

  if (SDA_PIN >= 0 && SCL_PIN >= 0) {
    Wire.begin(SDA_PIN, SCL_PIN);
  } else {
    Wire.begin();
  }

  if (!mux.begin()) {
    Serial.println("Failed to initialize I2C multiplexer at 0x70");
  }

  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    mux.selectChannel(SENSOR_CHANNELS[i]);
    if (!sensors[i].begin()) {
      Serial.print("Warning: sensor "); Serial.print(i); Serial.println(" failed to start (replace with actual VL53L4CD library init)");
    }
  }

  // return MUX to default
  mux.selectChannel(0);

  leftMotor.begin();
  rightMotor.begin();

  Serial.println("Setup complete");
}

void loop() {
  int distances[SENSOR_COUNT];
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    mux.selectChannel(SENSOR_CHANNELS[i]);
    distances[i] = sensors[i].readDistance();
    if (distances[i] < 0) {
      Serial.print("Sensor"); Serial.print(i); Serial.println(": no reading");
    } else {
      Serial.print("Sensor "); Serial.print(i); Serial.print(": "); Serial.print(distances[i]); Serial.println(" mm");
    }
  }

  // Basic decision logic (assumes sensors 0=left,1=front,2=right)
  int left = distances[0];
  int front = distances[1];
  int right = distances[2];

  bool didAction = false;

  if (front > 0 && front <= STOP_THRESHOLD) {
    Serial.println("Action: STOP (tug both)");
    leftMotor.tugStop();
    rightMotor.tugStop();
    didAction = true;
  } else if (left > 0 && right > 0) {
    if (left + 200 < right && left <= TURN_THRESHOLD) {
      Serial.println("Action: TURN LEFT (tug left)");
      leftMotor.tugOnce();
      didAction = true;
    } else if (right + 200 < left && right <= TURN_THRESHOLD) {
      Serial.println("Action: TURN RIGHT (tug right)");
      rightMotor.tugOnce();
      didAction = true;
    }
  }

  if (!didAction) {
    // idle state
  }

  delay(200); // main loop cadence
}
