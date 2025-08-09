# esp32s3_4_ir_array_line_following_obstical_avoiding

## Overview

This project features a line-following and obstacle-avoiding robot based on the ESP32-S3 microcontroller, equipped with a 4-IR sensor array. The robot is designed for autonomous navigation: it follows a path marked by a line and avoids obstacles using ultrasonic sensors.

---

## Robot Preview

![Robot Zeus](images/robot_zeus.jpg)

*Figure: "Zeus" — ESP32-S3 based line-following and obstacle-avoiding robot.*

---

## Features

- **ESP32-S3 Microcontroller**: Powerful, supports WiFi and BLE.
- **4 IR Sensor Array**: For accurate line detection and tracking.
- **Dual Ultrasonic Sensors**: For precise obstacle detection and avoidance.
- **Autonomous Navigation**: Follows a line and maneuvers around obstacles.
- **Expandable Design**: Easily adaptable for further robotics experiments.

---

## Hardware Requirements

- ESP32-S3 development board
- 4x IR sensors (for line tracking)
- 2x HC-SR04 ultrasonic sensors (for obstacle avoidance)
- Motor driver (e.g., L298N or TB6612FNG)
- 2x DC motors with wheels
- Chassis and battery pack
- Miscellaneous: wires, breadboard or PCB, mounting hardware

---

## Software Requirements

- Arduino IDE or PlatformIO
- ESP32 board support package
- Required libraries (e.g., Wire, NewPing, Adafruit_Sensor)

---

## How It Works

1. **Line Following**: The IR sensor array detects the track line. The ESP32-S3 processes sensor input and adjusts the motors to stay aligned.
2. **Obstacle Avoidance**: When an obstacle is detected by the ultrasonic sensors, the robot stops, determines a clear path (left/right/reverse), and continues following the line.
3. **Motor Control**: The ESP32-S3 sends commands to the motor driver for precise movement.

---

## Example Wiring Diagram

```
[ESP32-S3] -> [IR Sensors] -> [Ultrasonic Sensors] -> [Motor Driver] -> [Motors]
```
Refer to your hardware's datasheet and the Arduino wiring reference for detailed pin connections.

---

## Example Code Skeleton

```cpp
#include <Arduino.h>
// Include necessary libraries

void setup() {
  // Initialize sensors, motors, serial communication, etc.
}

void loop() {
  // Read IR sensor values
  // Check for obstacles using ultrasonic sensors
  // Line following and obstacle avoidance logic
}
```

---

## Applications

- Robotics education
- DIY and hobby robotics
- Autonomous vehicle research

---

## License

This project is open-source under the MIT License.

---

Feel free to contribute or modify for your own use!
