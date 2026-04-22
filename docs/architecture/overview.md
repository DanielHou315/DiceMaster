# System Architecture Overview

## Summary

DiceMaster runs on a Raspberry Pi 4 that hosts a ROS2 Humble node graph. The graph connects an IMU sensor pipeline, a game strategy layer, and three SPI screen bus managers. Each bus manager drives one ESP32-S3 board, which in turn drives two 480x480 IPS displays. All inter-component communication uses typed ROS2 pub/sub topics.

## Component Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi Central                        │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │ Game Manager │    │   Chassis    │    │     IMU      │     │
│  │    Node      │    │    Node      │    │   Pipeline   │     │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘     │
│         │                   │                    │              │
│         │ /screen_X_cmd     │ /chassis/*         │ /imu/*       │
│         │                   │                    │              │
│  ┌──────▼───────────────────▼────────────────────▼─────┐       │
│  │           Screen Bus Managers (3 instances)         │       │
│  │   Bus 0    │    Bus 1    │    Bus 3                │       │
│  │ (Scr 1,6)  │  (Scr 3,5)  │  (Scr 2,4)              │       │
│  └──────┬──────────────┬──────────────┬────────────────┘       │
│         │              │              │                         │
│         │ SPI          │ SPI          │ SPI                     │
└─────────┼──────────────┼──────────────┼─────────────────────────┘
          │              │              │
    ┌─────▼──┐     ┌─────▼──┐     ┌─────▼──┐
    │ ESP32  │     │ ESP32  │     │ ESP32  │
    │ Screen │     │ Screen │     │ Screen │
    │  1 & 6 │     │  3 & 5 │     │  2 & 4 │
    └────────┘     └────────┘     └────────┘
```

## Node Graph

The main launch file (`dicemaster.launch.py`) composes four sub-launches:

```
dicemaster.launch.py
├── imu.launch.py
│   ├── imu_hardware         — reads MPU6050 over I2C, publishes /imu/data_raw
│   ├── imu_filter_madgwick  — third-party Madgwick filter, publishes /imu/data
│   └── motion_detector      — analyzes filtered data, publishes /imu/motion
├── chassis.launch.py
│   └── chassis              — tracks orientation, publishes /chassis/* topics
├── screens.launch.py
│   ├── screen_bus_manager_0 — SPI bus 0, drives screens 1 and 6
│   ├── screen_bus_manager_1 — SPI bus 1, drives screens 3 and 5
│   └── screen_bus_manager_3 — SPI bus 3, drives screens 2 and 4
└── managers.launch.py
    └── game_manager         — discovers and loads strategy nodes dynamically
        └── strategy_{name}  — active game (user-defined BaseStrategy subclass)
```

## ROS2 Topic Map

| Topic | Type | Published By | Consumed By |
|---|---|---|---|
| `/imu/data_raw` | `sensor_msgs/Imu` | `imu_hardware` | `imu_filter_madgwick` |
| `/imu/data` | `sensor_msgs/Imu` | `imu_filter_madgwick` | `motion_detector`, `chassis` |
| `/imu/motion` | `MotionDetection` | `motion_detector` | active strategy |
| `/chassis/orientation` | `ChassisOrientation` | `chassis` | active strategy |
| `/chassis/screen_{1-6}_pose` | `ScreenPose` | `chassis` | `screen_bus_manager_*` |
| `/screen_{1-6}_cmd` | `ScreenMediaCmd` | active strategy | `screen_bus_manager_*` |
| `/game_control` (service) | `DiceGameControl` | — | `game_manager` |

## Data Flow: IMU to Display

The full path from physical motion to pixels on a screen:

```
1. MPU6050 (I2C)
   └─ Raw accelerometer + gyroscope readings at configured polling rate

2. imu_hardware node
   └─ Publishes sensor_msgs/Imu to /imu/data_raw

3. imu_filter_madgwick (third-party ROS2 node)
   └─ Applies Madgwick quaternion filter
   └─ Publishes sensor_msgs/Imu with orientation to /imu/data

4. motion_detector node
   └─ Detects shake events, rotation events, intensity thresholds
   └─ Publishes MotionDetection to /imu/motion

5. chassis node (parallel)
   └─ Reads orientation quaternion from /imu/data
   └─ Determines which screen face is pointing up / down
   └─ Publishes ChassisOrientation and per-screen ScreenPose topics

6. Strategy node (active game)
   └─ Subscribes to /imu/motion and/or /chassis/orientation
   └─ Game logic decides what content to show and on which screens
   └─ Publishes ScreenMediaCmd to /screen_{id}_cmd

7. ScreenBusManager node (one per SPI bus)
   └─ Subscribes to /screen_{id}_cmd for its two screens
   └─ Queues commands with priority (PriorityQueue)
   └─ Reads asset file (JSON text / JPEG image / .gif.d directory)
   └─ Converts to binary protocol messages (see docs/protocol.md)
   └─ Splits large payloads into 2 KB chunks
   └─ Transmits via SPI with DMA-aligned padding (4-byte boundaries)

8. ESP32-S3 firmware
   └─ Parses SPI packet header (SOF, message type, screen ID bitmask)
   └─ Filters by screen ID — ignores packets not addressed to it
   └─ Reassembles image chunks; invalidates if timeout exceeded
   └─ Renders content: JPEG decode (JPEGDEC), text (U8g2), GIF (frame loop)
   └─ Drives 480x480 IPS display with correct rotation
```

## Game Manager and Strategy Lifecycle

The `game_manager` node owns the lifecycle of strategy nodes. On startup it scans the configured games directory, discovers all valid `BaseStrategy` subclasses, and launches the default game. The sequence when switching games:

1. Caller invokes the `/game_control` ROS2 service with a `STOP` command for the current game and a `START` command for the new game
2. `game_manager` calls `stop_strategy()` on the current strategy instance
3. The current strategy node is removed from the `MultiThreadedExecutor`
4. A new strategy instance is constructed for the requested game (which calls `start_strategy()` automatically)
5. The new strategy node is added to the executor and begins publishing screen commands

This design means screen bus managers and the IMU pipeline are never restarted during a game switch.

## SPI Protocol Summary

Each SPI message from the Pi to an ESP32 starts with a 5-byte header:

| Byte | Field | Notes |
|---|---|---|
| 0 | Start of Frame | Always `0x7E` |
| 1 | Message Type | Text, Image Start, Image Chunk, Backlight, Ping, etc. |
| 2 | Screen ID | Bitmask: bit k targets screen k. Allows multi-screen broadcast. |
| 3–4 | Payload Length | Big-endian uint16 |

Text content, image chunks, and control commands (backlight, ping) each have distinct payload formats. See `/Users/danielhou/Code/DiceMaster/docs/protocol.md` for the full specification.

## Configuration

System behavior is controlled by `config.py` in the `dicemaster_central` package. Key configuration classes:

- `DiceConfig` — top-level, references all sub-configs
- `SPIBusConfig` — bus ID and chip select per SPI bus
- `ScreenConfig` — screen ID, bus assignment, default orientation
- `IMUConfig` — I2C bus and address, calibration values, polling rate
- `GameConfig` — games directory path, default game name

## Repository Structure

```
DiceMaster/
├── DiceMaster_Central/       — Raspberry Pi ROS2 codebase (submodule)
│   ├── src/dicemaster_central/
│   │   ├── dicemaster_central/
│   │   │   ├── games/        — BaseStrategy, game manager
│   │   │   ├── hw/           — SPI, I2C, IMU hardware drivers
│   │   │   ├── managers/     — ScreenBusManager
│   │   │   └── media_typing/ — Protocol implementation
│   │   └── dicemaster_central_msgs/  — Custom ROS2 message definitions
│   ├── examples/             — Example games and strategies
│   └── launch/               — ROS2 launch files
├── DiceMaster_ESPScreen/     — ESP32 Arduino firmware (submodule)
├── DiceMaster_HW/            — Hardware design files (submodule)
├── DiceMaster_Studio/        — Game authoring tools (submodule)
└── docs/                     — Project-level documentation
    ├── architecture/         — System architecture docs
    ├── decisions/            — Architecture Decision Records
    └── product/              — Product overview and personas
```
