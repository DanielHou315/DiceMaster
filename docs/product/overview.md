# DiceMaster Product Overview

## What Is DiceMaster?

DiceMaster is a programmable, multi-screen educational device shaped like a large dice. Each of its six faces carries a 480x480 IPS display, and the physical object responds to being shaken, rotated, and flipped. Educators write Python games that react to those motions — showing vocabulary words, quiz questions, image prompts, or animated feedback — without needing to understand the embedded systems underneath.

The core insight is that physical manipulation is more engaging than tapping a screen. Students interact with DiceMaster by picking it up, shaking it to advance to the next question, and turning it over to reveal an answer. The interaction is inherently collaborative: one device, used in the center of a group.

## Current Status

DiceMaster is in **Alpha**. The hardware and software stack is functional and has been demonstrated publicly. Active development continues on game tooling, documentation, and deployment ergonomics.

DiceMaster was presented at **SeriousPlay 2025**, where it was evaluated by educators and researchers in game-based learning. Early feedback from alpha testing focused on the beginner game development experience and multi-language content support.

## Hardware Summary

| Component | Specification |
|---|---|
| Central compute | Raspberry Pi 4 |
| Displays | 6x 480x480 IPS, full-color |
| Display controllers | 3x ESP32-S3, each driving 2 screens over SPI |
| Motion sensor | MPU6050 6-axis IMU (accelerometer + gyroscope) |
| Display–Pi link | SPI buses 0, 1, 3 (one per ESP32) |
| Sensor–Pi link | I2C |
| Enclosure | Custom 3D-printed dice form factor |

## Software Summary

| Layer | Technology |
|---|---|
| OS | Debian Bookworm (Raspberry Pi) |
| Middleware | ROS2 Humble |
| Game API | Python 3.11, BaseStrategy pattern |
| IMU filtering | Madgwick filter (imu_filter_madgwick, ROS2) |
| ESP32 firmware | Arduino C++ (GFX, JPEGDEC, U8g2) |
| Asset formats | JSON (text), JPEG (images), GIF directory (animation) |

## Key Capabilities

**Six independent screens.** Each face can display different content simultaneously. A vocabulary game might show a question on the top face, an answer on the bottom, and four image hints on the sides. Screens update with orientation awareness — if you rotate the dice 90 degrees, text re-renders upright.

**Motion-driven interaction.** The MPU6050 IMU feeds raw sensor data through a Madgwick filter for orientation estimation. The motion detector node publishes a `/imu/motion` topic with shake detection, rotation events, and intensity values. Game logic subscribes to this topic directly.

**Programmable game system.** Any educator or developer can write a game by subclassing `BaseStrategy` in Python and implementing two methods: `start_strategy()` and `stop_strategy()`. The framework handles ROS2 node management, asset loading, and screen bus communication transparently.

**Multi-language text rendering.** The ESP32 firmware uses the U8g2 library with bundled fonts covering Arabic, Chinese, Cyrillic, Devanagari, and Latin scripts. Text assets are JSON files specifying content, font, color, and position — no image conversion needed.

**Hot-swappable games.** The game manager node can load a new strategy at runtime via a ROS2 service call without restarting the IMU pipeline or screen managers.

## Target Users

**Educators** — university instructors, language teachers, and learning designers who want to create interactive activities for their students. They need a simple Python API, clear documentation, and working examples. They are not expected to know ROS2, SPI protocols, or embedded systems.

**Developers** — software and hardware engineers building custom games for research labs or commercial applications. They want full access to the ROS2 topic interface, screen bus addressing, motion event data, and the ability to integrate external systems.

## Value Proposition

DiceMaster removes the hardware and middleware barrier between an educator's pedagogical idea and a working interactive artifact. A language teacher who can write a Python list can create a shake-to-quiz vocabulary game in under an hour, deploy it to the physical device, and use it in class the same day. For developers, the full ROS2 topic graph and a modular architecture provide a solid foundation for custom extensions.
