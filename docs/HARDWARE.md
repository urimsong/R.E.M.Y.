Hardware wiring and notes

Overview
- ESP32-S3 DevKit (Arduino core)
- Three VL53L4CD Time-of-Flight distance modules (same I2C address)
- TCA9548A I2C multiplexer to talk to three identical I2C sensors
- Two MG996R servo motors attached to backpack straps for tugging

Wiring
- TCA9548A:
  - VCC -> 3.3V (or 5V depending on board; check module)
  - GND -> GND
  - SDA -> ESP32 SDA pin
  - SCL -> ESP32 SCL pin

- VL53L4CD sensors:
  - For each sensor, connect SDA/SCL to the corresponding channel pins on the TCA9548A (channels 0,1,2)
  - VCC -> 3.3V (follow sensor module recommendation)
  - GND -> GND
  - If the sensor module has an XSHUT or reset pin, you can wire it to separate GPIOs to assign addresses instead of using a multiplexer.

- MG996R servos:
  - VCC -> 5V supply capable of providing servo current (do NOT power servos from ESP32 3.3V regulator)
  - GND -> common ground with ESP32
  - Signal -> chosen GPIO pins (see `platformio.ini` code pins)

Power notes
- Servos can draw significant current under load. Use a dedicated 5V supply (e.g., battery or UBEC) sized for stall current.
- Keep grounds common between servo supply and ESP32.

I2C notes
- The code uses a TCA9548A multiplexer at address 0x70. If your module uses a different address, update `MUX_ADDR`.
- Alternatively, if your sensor modules support address reprogramming (via XSHUT / address pin), you can avoid the multiplexer.

Pin mapping (example)
- Left servo signal -> GPIO16
- Right servo signal -> GPIO17

Safety
- Add an emergency-stop physical switch into the motor power line.
- Add software timeouts and watchdog timers in the firmware to stop motors if sensor readings fail.
