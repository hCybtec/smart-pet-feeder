# Smart Pet Feeder Project

This repository contains the code for the IoT-enabled Smart Pet Feeder project. It uses:
- ESP32 and Blynk for remote servo control.
- TBeam microcontroller for motion detection, food level monitoring, and proximity-based activation.

## Files
- `servo_blynk.ino`: Arduino C++ code for Blynk and ESP32 integration.
- `smart_feeder_tbeam.cpp`: TBeam code with PIR, ultrasonic sensors, and servo management.

## Features
- Servo control via the Blynk app.
- LCD and buzzer alerts for low food levels.
- Proximity-based activation for pet feeding.

## Libraries Required
- `BlynkSimpleEsp32.h`
- `ESP32Servo.h`
- `Servo.h`
- `LiquidCrystal_I2C.h`
