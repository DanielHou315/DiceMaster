# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

DiceMaster is a programmable multi-screen dice prototype designed for language learning. The system consists of:
- **Raspberry Pi** running ROS2 for central control
- **ESP32 screen modules** (6 faces) communicating via SPI
- Custom hardware including IMU sensors for orientation detection

This is a monorepo with git submodules for each component.

## Project Structure

### Submodules

- **DiceMaster_Central**: Raspberry Pi codebase (ROS2 Python package)
- **DiceMaster_ESPScreen**: ESP32 Arduino firmware for screen drivers
- **DiceMaster_ROS_workspace**: ROS2 workspace for building and development
- **DiceMaster_Studio**: (deprecated) GUI tool for game design

### Key Components in DiceMaster_Central

- `dicemaster_central/hw/screen/`: Screen communication over SPI
- `dicemaster_central/hw/imu/`: IMU hardware and motion detection
- `dicemaster_central/hw/chassis.py`: Robot pose transforms and screen orientation (46-line node)
- `dicemaster_central/games/`: Game base classes and strategy system
- `dicemaster_central/managers/game_manager.py`: Game discovery and lifecycle management
- `dicemaster_central/media_typing/`: Protocol definitions and media types
- `launch/`: ROS2 launch files for different subsystems

## Build and Development

### Building the ROS2 Workspace

```bash
cd DiceMaster_ROS_workspace
source prepare.sh
colcon build --symlink-install
```

The `defaults.json` file configures colcon with symlink install and Release build type.

### Running Tests

```bash
# From DiceMaster_Central directory after sourcing ROS2
cd DiceMaster_Central
python3 -m pytest tests/
```

Test files are in `DiceMaster_Central/dicemaster_central/tests/`:
- `test_protocol.py`: Protocol encoding/decoding tests
- `test_screen.py`: Screen publisher tests
- `test_remote_logger.py`: Logger tests
- `test_spi2.py`: SPI communication tests

### Launch Files

Main launch files in `DiceMaster_Central/dicemaster_central/launch/`:

```bash
# Launch complete system
ros2 launch dicemaster_central dicemaster.launch.py

# Launch specific subsystems
ros2 launch dicemaster_central imu.launch.py
ros2 launch dicemaster_central screens.launch.py
ros2 launch dicemaster_central chassis.launch.py
ros2 launch dicemaster_central managers.launch.py
```

## Architecture

### Communication Protocol

The Raspberry Pi and ESP32 screens communicate via SPI using a custom packet protocol (see `docs/protocol.md`):

- **Header**: 5 bytes (SOF marker 0x7E, message type, screen ID with CAN-bus-like bit masking, payload length)
- **Screen ID**: Bit-masked (bit k corresponds to screen k) for multi-screen addressing
- **Message types**: Text batches, image chunks, backlight control, ping/acknowledgment
- **Images**: Chunked transfer with automatic timeout-based invalidation (100ms × num_chunks)

### Game System

Games are discovered from the examples directory. Each game requires:
- `config.json`: Game name, strategy reference, and strategy configuration
- `assets/`: Directory containing game media (images, JSON data)

The game manager (`game_manager.py`) discovers strategies and games, then instantiates strategy nodes that inherit from `BaseStrategy`. Strategies control game logic and screen content.

### Hardware Management

- **IMU**: Motion detection publishes to ROS topics, consumed by chassis node
- **Chassis**: Publishes TF transforms and screen orientation based on IMU data
- **Screen Bus Manager**: Manages SPI communication to all 6 screens via bit-masked addressing
- Screen IDs (0-6) map to colors: Red, Green, Blue, Yellow, Magenta, Cyan

## Dependencies

### Raspberry Pi (ROS2 Humble)

Key Python packages (from `requirements.txt`):
- numpy, pillow, opencv
- spidev (custom build needed: https://forums.raspberrypi.com/viewtopic.php?t=124472)
- smbus2, scipy

ROS2 dependencies (from `package.xml`):
- rclpy, geometry_msgs, sensor_msgs, tf2_ros
- dicemaster_central_msgs (custom message package)

### ESP32 Screens

Arduino libraries (from `docs/software.md`):
- esp32 (2.0.17)
- ESP32DMASPI (0.6.5)
- GFX Library for Arduino (1.4.9)
- JPEGDEC (1.8.2)
- U8g2 (2.35.30)

## Raspberry Pi Setup Notes

- Username must be `dice` for auto-start scripts
- ROS2 humble installed in `/home/dice/ros2_humble` (sourced in `.bashrc`)
- SPI buffer size requires custom py-spidev (2048 byte MOSI buffer for images)
- Enable SPI and I2C via `raspi-config` interface options
- Follow `DiceMaster_Central/README.md` for complete deployment setup

## ESP32 Factory Reset

If the board crashes and can't receive USB uploads, drag the factory UF2 file from `resources/` to reset.

## File Naming Convention

Entry points are named with `.py` extension explicitly (e.g., `screen_bus_manager.py`, `imu_hardware.py`) as defined in `setup.py`.