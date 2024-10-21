# OmniCtrl: The coolest robot controller 🕹

The controller for Doug, the omni-wheeled robot!

[![rover gif](https://github.com/user-attachments/assets/3abf20fa-31b0-4424-8c4f-44cd74c5599f)](https://makerworld.com/en/models/719381)

_GIF of the robot in action!_

See my [MakerWorld design](https://makerworld.com/en/models/719381) for more details!

_Join my [3d desin discord](https://3d.t413.com/go/discord?gh.omni) and say hi and talk shop!_


## Features:

- Designed to be a universal toolbox of robotics control
- Works with different CanBus motors!
  * All using the native ESP32 TWAI CanBus library
  * Allows different motors on the same shared CAN bus. Right now:
    - ODrive
    - Xiaomi CyberGear motors
- Supports different radio protocols
  * CRSF serial systems: ExpressLRS & Crossfire
  * ESP-NOW for low-latency control without external hardware
- IMUs!
  * Right now using the M5Stack IMU abstractions
  * Uses a Madgwick filter for orientation estimation
  * Fancy rotation matrix for on-end balancing
- LCD support using M5Stack library
  * has system for tuning PID gains

## Installation

- Clone the repo
- Open in PlatformIO
  * Either from command-line or, recommended:
  * VSCode. Install the PlatformIO extension and open the project
- Choose the target you want and just hit upload!

## Hardware: Rover

- Microcontroller: M5Stack AtomS3 [[M5Stack](https://shop.m5stack.com/products/atoms3-dev-kit-w-0-85-inch-screen)]
- CANBus transceiver: Recommend the M5Stack Mini CAN Unit [[M5Stack](https://shop.m5stack.com/products/mini-can-unit-tja1051t-3)]


## Hardware: ESP-NOW Controller

- M5StickC PLUS2 [[M5Stack](https://shop.m5stack.com/products/m5stickc-plus2-esp32-mini-iot-development-kit)]
- I2C Joystick 2 [[M5Stack](https://shop.m5stack.com/products/i2c-joystick-2-unit-stm32g030)]

