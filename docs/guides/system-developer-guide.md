# DiceMaster System Developer Guide

**Audience**: Hardware engineers, software maintainers, and contributors who need to understand, modify, or extend the full DiceMaster system.

**Scope**: This is the master-repo guide. It gives the cross-component picture and links to submodule docs for depth. For game creation (educator-facing API), see [docs/beginner_game_guide.md](../beginner_game_guide.md).

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Repository Structure](#2-repository-structure)
3. [Development Environment Setup](#3-development-environment-setup)
4. [Component Deep-Dives](#4-component-deep-dives)
5. [SPI Communication Protocol](#5-spi-communication-protocol)
6. [IMU Pipeline](#6-imu-pipeline)
7. [Testing](#7-testing)
8. [Common Maintenance Tasks](#8-common-maintenance-tasks)
9. [Architecture Decision Records Index](#9-architecture-decision-records-index)
10. [Getting Help](#10-getting-help)

---

## 1. System Overview

### Full Component Map

DiceMaster is a six-faced physical dice with a 480x480 IPS display on each face. A Raspberry Pi 4 runs the game engine and sensor pipeline; three ESP32-S3 microcontrollers render display content; a custom PCB routes power and SPI signals; a web app (Studio) lets educators author games without physical hardware.

```
┌─────────────────────────────────────────────────────────────────┐
│                     Raspberry Pi (Central)                      │
│                                                                  │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐     │
│  │ Game Manager │    │   Chassis    │    │     IMU      │     │
│  │  + Strategy  │    │    Node      │    │   Pipeline   │     │
│  └──────┬───────┘    └──────┬───────┘    └──────┬───────┘     │
│         │ /screen_X_cmd     │ /chassis/*         │ /imu/*       │
│  ┌──────▼───────────────────▼────────────────────▼─────┐       │
│  │     Screen Bus Managers (3 instances — SPI)         │       │
│  └──────┬──────────────┬──────────────┬────────────────┘       │
└─────────┼──────────────┼──────────────┼─────────────────────────┘
          │ SPI Bus 0    │ SPI Bus 1    │ SPI Bus 3
    ┌─────▼──┐     ┌─────▼──┐     ┌─────▼──┐
    │ ESP32  │     │ ESP32  │     │ ESP32  │
    │ Scr 1,6│     │ Scr 3,5│     │ Scr 2,4│
    └────────┘     └────────┘     └────────┘
          ↕ SPI          ↕ SPI          ↕ SPI
    ┌─────────────────────────────────────────┐
    │       DiceMaster_HW  (PCB / KiCad)      │
    └─────────────────────────────────────────┘

  Browser (Studio / Central_Web)
  ┌─────────────────────────────────────────┐
  │  DiceMaster_Studio  (Vite + TypeScript)  │
  │  ↕ postMessage  ↕                        │
  │  DiceMaster_Central_Web  (dice Python    │
  │  SDK running in Pyodide)                 │
  └─────────────────────────────────────────┘
```

### What Each Repo Contains

| Repo | Language | Purpose |
|------|----------|---------|
| `DiceMaster_Central` | Python / ROS2 | Game engine, IMU pipeline, SPI managers, BaseStrategy API |
| `DiceMaster_ESPScreen` | Arduino C++ | Firmware for ESP32-S3; parses SPI packets, renders to displays |
| `DiceMaster_HW` | KiCad | PCB schematic and layout for the dice carrier board |
| `DiceMaster_Central_Web` | Python (Pyodide) | Browser-compatible `dice` SDK mirroring the hardware API |
| `DiceMaster_Studio` | TypeScript / Vite | Web-based game simulator and authoring tool |

Repos are separated because they have incompatible build systems (ROS2 colcon, Arduino IDE, KiCad, npm), different deployment targets (RPi, ESP32 flash, PCB fab, browser), and different release cadences. A change to the PCB layout does not need to trigger a Python or firmware build.

### Two Runtime Environments

**Physical hardware**: Raspberry Pi runs `dicemaster.launch.py`, which starts the full ROS2 node graph. Real IMU data drives the game. Screens are physical 480x480 IPS panels connected through the PCB.

**Web simulator**: DiceMaster_Studio serves a browser app. Game Python code runs inside Pyodide (a WebAssembly Python runtime). Simulated shake and orientation events are injected via `postMessage`. Screen output renders in SVG/canvas panels. No physical hardware is required. This is the primary environment for educators writing games.

---

## 2. Repository Structure

### Submodule Layout

The root `DiceMaster` repository is a meta-repo (sometimes called a "superproject"). All substantive code lives in submodules:

```
DiceMaster/                        ← meta-repo (this guide lives here)
├── .gitmodules                    ← declares all five submodules
├── docs/                          ← project-level docs (ADRs, protocol spec, this guide)
│   ├── architecture/overview.md
│   ├── decisions/                 ← Architecture Decision Records (ADRs)
│   ├── guides/                    ← Developer guides (including this file)
│   └── protocol.md                ← Full SPI protocol specification
├── DiceMaster_Central/            ← submodule: ROS2 codebase
│   ├── src/dicemaster_central/    ← main Python package
│   │   ├── games/                 ← BaseStrategy, game manager
│   │   ├── hw/                    ← SPI, I2C, IMU hardware drivers
│   │   ├── managers/              ← ScreenBusManager
│   │   └── media_typing/          ← protocol.py (Pi-side encoding)
│   ├── src/dicemaster_central_msgs/ ← custom ROS2 message definitions
│   ├── examples/                  ← example games and strategies
│   ├── launch/                    ← ROS2 launch files
│   └── docs/                      ← Central-specific docs (setup, API, runbooks)
├── DiceMaster_ESPScreen/          ← submodule: ESP32 Arduino firmware
│   ├── Dicemaster/                ← main .ino sketch + headers
│   └── docs/
├── DiceMaster_HW/                 ← submodule: KiCad hardware files
│   ├── PCB/                       ← KiCad project files
│   └── docs/
├── DiceMaster_Central_Web/        ← submodule: browser Python SDK
│   ├── dice/                      ← Python package (runs in Pyodide)
│   └── docs/
└── DiceMaster_Studio/             ← submodule: Vite web app
    ├── src/                       ← TypeScript + React source
    └── docs/
```

### Cloning

Always clone with `--recursive` to initialize all submodules in one step:

```bash
git clone --recursive https://github.com/DanielHou315/DiceMaster.git
```

If you already cloned without `--recursive`:

```bash
git submodule update --init --recursive
```

### How Submodules Relate

The meta-repo records a specific commit SHA for each submodule. When you `cd DiceMaster_Central` you are on a detached HEAD pinned to that SHA. To advance a submodule to its latest remote commit:

```bash
git submodule update --remote --merge
```

Then commit the updated SHA in the meta-repo. This is the correct way to "bump" a submodule. Do not push changes only to the submodule without also updating the meta-repo pointer, or other developers will be on the old version.

Changes to `dicemaster_central_msgs` (message definitions) must be coordinated with `DiceMaster_Central`: messages are compiled during `colcon build` and the generated Python and C++ stubs are consumed by all nodes in the same workspace.

---

## 3. Development Environment Setup

### macOS Local Dev (no hardware required)

This workflow is used for editing, code review, and running unit tests that do not touch hardware.

```bash
# Clone the repo
git clone --recursive https://github.com/DanielHou315/DiceMaster.git
cd DiceMaster

# Python dependencies for DiceMaster_Central (tests run without ROS2)
cd DiceMaster_Central/src/dicemaster_central
pip install -r requirements.txt

# Run protocol unit tests locally (no ROS2 needed)
python3 -m pytest tests/ -v

# Studio (web simulator) — requires Node.js
cd ../../../DiceMaster_Studio
npm install
npm run dev        # starts Vite dev server at localhost:5173

# Central_Web Python SDK (Pyodide target, test locally with CPython)
cd ../DiceMaster_Central_Web
pip install uv
uv sync
uv run pytest tests/ -v
```

ROS2 itself does not run on macOS. Protocol encoding/decoding tests, strategy unit tests, and the web simulator all run without it.

### Remote Dev on Raspberry Pi (full system)

The remote target is the Raspberry Pi at SSH host `dice1` (configured in `~/.ssh/config`). ROS2 Humble is installed at `~/ros2_humble/` on the Pi. The project is cloned at `~/DiceMaster/`.

#### Initial setup (one-time, already done on dice1)

See [DiceMaster_Central/docs/setup/rpi_setup.md](../../DiceMaster_Central/docs/setup/rpi_setup.md) for the full procedure. Key steps:

1. Flash Raspbian OS 64-bit Lite; username `dice`; enable SSH, SPI, I2C
2. Build ROS2 Humble from source into `~/ros2_humble/` (multi-hour, already done)
3. Install custom `py-spidev` with 8 KB buffer support (standard build caps at 4 KB)
4. Build the workspace with `colcon build --symlink-install`

#### The 5-Step Development Loop

```
Edit locally (macOS)
  → Commit & push (submodule + meta-repo)
    → Pull on remote
      → Rebuild if needed
        → Test on remote
```

Step by step:

**1. Edit locally** — Edit files in `/Users/danielhou/Code/DiceMaster/<submodule>/`.

**2. Commit and push** — Commit in the submodule first, then update the meta-repo pointer:

```bash
# In the submodule directory (e.g., DiceMaster_Central)
git add <files>
git commit -m "your message"
git push

# In the meta-repo root
git add DiceMaster_Central
git commit -m "bump DiceMaster_Central"
git push
```

**3. Pull on the Pi**:

```bash
ssh dice1 'cd ~/DiceMaster/DiceMaster_Central && git pull'
```

Because `--symlink-install` was used during the initial build, Python source files are symlinked into the ROS2 install tree. Pulling updates the source directly — no rebuild is needed for Python-only changes.

**4. Rebuild if needed** — Only required when any of these change:

| Changed file | Rebuild required? |
|---|---|
| `*.py` Python source | No |
| `*.launch.py` launch files | No |
| `config.py` | No (restart nodes to apply) |
| `setup.py` or `package.xml` | Yes |
| `*.msg` or `*.srv` message definitions | Yes |
| C++ source | Yes |

```bash
ssh dice1 'cd ~/DiceMaster/DiceMaster_Central && source ~/ros2_humble/install/setup.bash && colcon build --symlink-install'
```

**5. Test on remote**:

```bash
# Unit tests
ssh dice1 'cd ~/DiceMaster/DiceMaster_Central/src/dicemaster_central && python3 -m pytest tests/'

# Full system (manual launch)
ssh dice1 'source ~/ros2_humble/install/setup.bash && source ~/DiceMaster/DiceMaster_Central/install/setup.bash && ros2 launch dicemaster_central dicemaster.launch.py'

# Or via systemd
ssh dice1 'sudo systemctl restart dicemaster && sudo systemctl status dicemaster'
```

---

## 4. Component Deep-Dives

### DiceMaster_Central

The ROS2 game engine running on the Raspberry Pi. This is the largest and most central component. It contains four major subsystems in a single colcon workspace:

- **IMU pipeline** (`imu_hardware`, `imu_filter_madgwick`, `motion_detector` nodes): reads the MPU6050 IMU over I2C, applies the Madgwick quaternion filter, and publishes shake and orientation events on `/imu/motion` and `/chassis/*` topics.
- **Chassis node**: maps filtered IMU orientation to dice face geometry, publishing which screen is currently on top, on the bottom, and the rotation angle of each screen face.
- **Screen bus managers** (one per SPI bus): receive `ScreenMediaCmd` messages from the active game, encode them per the binary protocol, and transmit them over SPI to the ESP32 boards.
- **Game manager + BaseStrategy**: discovers Python strategy files at runtime via `importlib`, manages their lifecycle (start/stop), and provides the `/game_control` service for hot-swapping games without restarting the IMU or screen subsystems.

You would modify this component to: add support for a new sensor, define new ROS2 topics or message types, change IMU filtering parameters, add a new game API method to `BaseStrategy`, or fix a bug in SPI transmission logic.

Submodule docs: [DiceMaster_Central/docs/](../../DiceMaster_Central/docs/) — start with `docs/api/architecture.md` for the node graph, `docs/setup/rpi_setup.md` for deployment, and `docs/runbooks/deploy.md` for the deployment runbook.

### DiceMaster_ESPScreen

Arduino C++ firmware for the three ESP32-S3 boards. Each board manages two 480x480 IPS displays connected over a shared SPI bus on the ESP32 side. The firmware receives binary protocol packets from the Raspberry Pi over the Pi-to-ESP32 SPI link, parses the packet header to determine message type and target screen, and renders content using the Arduino GFX library (display driver), JPEGDEC (JPEG decoding), and U8g2 (multi-language font rendering). GIF playback is also handled on the ESP32, freeing the Pi from per-frame transmission.

The CAN-bus bitmask addressing scheme (bit k in the Screen ID byte targets screen k) lets the Pi broadcast to both screens on a bus in a single SPI transaction by setting two bits simultaneously.

You would modify this component to: add a new display type or resolution, add or change font sets, implement a new message type in the protocol (always paired with a change to `protocol.py` on the Pi side), change the chunk timeout behavior, or port to a different microcontroller.

Submodule docs: [DiceMaster_ESPScreen/docs/](../../DiceMaster_ESPScreen/docs/)

### DiceMaster_HW

KiCad 9.0 project files for the DiceMaster carrier PCB. The PCB routes SPI buses from the Raspberry Pi GPIO header to the three ESP32 board connectors, provides I2C breakout for the MPU6050 IMU, and distributes power. The repo is configured with the `kicad-mcp` MCP server, which allows Claude Code to inspect schematics and PCB layouts directly when working from this directory.

You would modify this component to: revise the PCB for a new hardware revision, add connectors for additional sensors or peripherals, change GPIO pin assignments (always coordinated with `config.py` in DiceMaster_Central), or prepare Gerber files for a board re-spin.

Submodule docs: [DiceMaster_HW/docs/](../../DiceMaster_HW/docs/) and [DiceMaster_HW/README.md](../../DiceMaster_HW/README.md) for the KiCad MCP setup.

### DiceMaster_Central_Web

A Python package named `dice` that mirrors the hardware `dicemaster_central` API but runs inside Pyodide (WebAssembly Python) in the browser. It provides modules for screen control (`dice.screen`), motion callbacks (`dice.motion`), orientation callbacks (`dice.orientation`), timers (`dice.timer`), asset access (`dice.assets`), and the same `BaseStrategy` base class as the hardware SDK. Communication between the Python layer and the JavaScript host is done via `postMessage` (outbound commands from Python to JS) and `worker.postMessage` (inbound events from JS to Python).

The API surface intentionally matches the hardware SDK so that a strategy written for the web simulator can run on physical hardware with only an import change.

You would modify this component to: add a new module or method to the student API, fix a Pyodide compatibility issue, add a new inbound event type from the JS host, or add tests for the bridge protocol.

Submodule docs: [DiceMaster_Central_Web/docs/](../../DiceMaster_Central_Web/docs/) and [DiceMaster_Central_Web/README.md](../../DiceMaster_Central_Web/README.md) for the JS bridge protocol.

### DiceMaster_Studio

A Vite + TypeScript/React web application that serves as the game simulator and authoring tool. It hosts a Pyodide runtime, loads student-written Python game strategies, simulates shake and orientation events through UI controls, and renders the six dice face screens visually. The Studio is the primary interface for educators who do not have physical hardware.

You would modify this component to: change the simulator UI layout, add new simulated event types, improve the visual screen rendering, add game asset management features, or change how Pyodide is initialized and communicated with.

Submodule docs: [DiceMaster_Studio/docs/](../../DiceMaster_Studio/docs/)

---

## 5. SPI Communication Protocol

### Physical Layer

The Raspberry Pi has four usable SPI buses (0, 1, 3; a fourth is reserved). Each bus runs as SPI master to one ESP32-S3 acting as SPI slave. The physical connection routes through the DiceMaster PCB. All SPI messages are padded to 4-byte boundaries for DMA alignment on both ends.

The standard `py-spidev` library caps SPI buffer size at 4 KB, which is insufficient for JPEG image chunks. A custom build of `py-spidev` with an 8 KB buffer is required on the Raspberry Pi (see [DiceMaster_Central/docs/setup/rpi_setup.md](../../DiceMaster_Central/docs/setup/rpi_setup.md)).

### Packet Structure

Every SPI message begins with a 5-byte header:

```
Byte 0:   SOF (Start of Frame) = 0x7E
Byte 1:   Message Type (see table below)
Byte 2:   Screen ID (bitmask — bit k targets screen k)
Byte 3-4: Payload length, big-endian uint16
Byte 5+:  Payload (varies by message type)
```

**Message types:**

| Value | Name | Description |
|---|---|---|
| `0x01` | `TEXT_BATCH` | Text display: background color, rotation, and N text items each with position/font/color |
| `0x02` | `IMAGE_TRANSFER_START` | Begin image: includes metadata (image ID, format, resolution, chunk count, rotation) plus chunk 0 data embedded |
| `0x03` | `IMAGE_CHUNK` | Subsequent image chunks (image ID, chunk ID, byte offset, chunk data) |
| `0x04` | `IMAGE_TRANSFER_END` | Deprecated; timeout-based invalidation is used instead |
| `0x05` | `BACKLIGHT_ON` | Turn backlight on |
| `0x06` | `BACKLIGHT_OFF` | Turn backlight off |
| `0x07` | `PING_REQUEST` | Check connectivity (no payload) |
| `0x08` | `PING_RESPONSE` | ESP32 response to ping |
| `0x09` | `ACKNOWLEDGMENT` | Success acknowledgment |
| `0x0A` | `ERROR_MESSAGE` | Error response with error code |

**CAN-bus screen addressing**: The Screen ID byte is a bitmask. Bit k corresponds to screen k. Setting `0x05` (binary `00000101`) targets screens 0 and 2 in a single transmission. An ESP32 silently drops packets where its screen's bit is not set (error code `SCREEN_ID_MISMATCH = 0x15`).

**Image chunking**: A 2 KB SPI buffer is used per transaction. A typical 10–15 KB JPEG is split into N chunks. The `IMAGE_TRANSFER_START` message embeds chunk 0 and declares the total chunk count. If the ESP32 does not receive all chunks within `100ms × N`, it discards the partial image — no explicit `IMAGE_TRANSFER_END` is needed.

For the full payload field specifications and error code table, see [docs/protocol.md](../protocol.md).

### Changes That Require Both Sides to Be Updated in Sync

The protocol is implemented in two places:
- **Pi side**: `DiceMaster_Central/src/dicemaster_central/dicemaster_central/media_typing/protocol.py` (Python)
- **ESP32 side**: `DiceMaster_ESPScreen/Dicemaster/protocol.h` (C++)

Any of the following changes require both files to be updated together and deployed atomically:
- Adding a new message type (new `0x0N` value)
- Changing the header structure (byte positions, lengths)
- Changing a payload field layout (e.g., adding a field to `IMAGE_TRANSFER_START`)
- Changing the SOF marker value
- Changing the Screen ID bitmask scheme

A protocol mismatch between the Pi and ESP32 causes silent rendering failures. When debugging display issues after a protocol change, attach a serial monitor to the ESP32 (115200 baud) to read its error codes.

---

## 6. IMU Pipeline

### Pipeline Stages

The full path from physical motion to game events:

```
MPU6050 (I2C, 6-axis accel + gyro)
    ↓
imu_hardware node
    — Reads raw accelerometer and gyroscope data over I2C
    — Publishes sensor_msgs/Imu to /imu/data_raw
    ↓
imu_filter_madgwick node  (third-party ROS2 package from imu_tools)
    — Applies Madgwick quaternion filter (gain=0.1, no magnetometer, ENU frame)
    — Publishes sensor_msgs/Imu with full orientation quaternion to /imu/data
    ↓
motion_detector node  (parallel consumer of /imu/data)
    — Analyzes linear acceleration magnitude for shake detection
    — Applies configurable intensity threshold
    — Publishes MotionDetection to /imu/motion
    ↓
chassis node  (parallel consumer of /imu/data)
    — Maps orientation quaternion to dice geometry
    — Determines which face is pointing up and down (gravity alignment)
    — Publishes ChassisOrientation and per-screen ScreenPose topics
```

### Tuning Motion Detection Thresholds

Shake sensitivity is controlled by parameters in `motion_detector.py`. The key fields in the published `MotionDetection` message are:

- `shaking` (bool): true when linear acceleration magnitude exceeds the shake threshold
- `shake_intensity` (float): normalized shake magnitude
- `rotation_*` (bool): true for each of the 6 cardinal rotation directions
- `rotation_intensity` (float): magnitude of the current rotation event

To tune thresholds, monitor the raw values first:

```bash
# On the Pi:
ros2 topic echo /imu/motion
```

Then adjust the threshold constants in `motion_detector.py`. Because Python files are symlink-installed, the change takes effect after restarting the node — no rebuild is needed. If the system is running under systemd: `sudo systemctl restart dicemaster`.

For Madgwick filter convergence behavior, adjust the `gain` parameter in `imu.launch.py`. A lower gain makes the orientation estimate more stable but slower to respond; a higher gain is more responsive but noisier.

### Adding New Sensor Types

To add a second IMU or a different sensor (e.g., a pressure sensor):

1. Write a new ROS2 node in `DiceMaster_Central/src/dicemaster_central/dicemaster_central/hw/` that reads the sensor and publishes on a new topic using an appropriate standard or custom message type.
2. If the sensor requires a new custom message, add a `.msg` file to `dicemaster_central_msgs/`, then run `colcon build` to regenerate the Python stubs.
3. Add the new node to the appropriate launch file (e.g., `imu.launch.py` for additional IMU sensors).
4. Expose the new topic to strategy authors by adding a subscription helper to `BaseStrategy` if the data is game-relevant, following the pattern of the existing `/imu/motion` and `/chassis/orientation` subscriptions.

---

## 7. Testing

### Unit Tests (run locally or on the Pi)

The main test suite lives in `DiceMaster_Central/src/dicemaster_central/tests/`. It does not require ROS2 to be running.

```bash
# From DiceMaster_Central/src/dicemaster_central/
python3 -m pytest tests/ -v
```

Key test files:

| File | What it covers |
|---|---|
| `test_protocol.py` | Protocol encoding/decoding, DMA alignment, all message types, error handling |
| `test_screen.py` | Screen media command publishing, test publisher helpers |
| `test_spi2.py` | SPI device communication (requires hardware — skip on macOS) |
| `test_remote_logger.py` | Remote logging functionality |

For the `DiceMaster_Central_Web` SDK:

```bash
cd DiceMaster_Central_Web
uv run pytest tests/ -v
```

### Hardware-in-the-Loop Tests

These require the physical dice connected and the full ROS2 graph running.

**Verify IMU pipeline**:
```bash
ros2 topic echo /imu/data_raw         # raw sensor data
ros2 topic echo /imu/data             # filtered with orientation
ros2 topic echo /imu/motion           # shake and rotation events
ros2 topic hz /imu/data               # check update rate
```

**Verify chassis**:
```bash
ros2 topic echo /chassis/orientation
ros2 topic echo /chassis/screen_1_pose
```

**Send a test command to one screen**:
```bash
ros2 topic pub /screen_1_cmd dicemaster_central_msgs/msg/ScreenMediaCmd \
  "{screen_id: 1, media_type: 0, file_path: '/path/to/greeting.json'}" --once
```

**Verify all nodes are running** (expected minimum set after full launch):
```bash
ros2 node list
# /dice_chassis_node
# /game_manager
# /imu_hardware
# /motion_detector
# /screen_bus_manager_0
# /screen_bus_manager_1
# /screen_bus_manager_3
```

**Run the pipeline_test diagnostic game** (cycles all screens):
```bash
# Set default_game = "test" in config.py, then:
ros2 launch dicemaster_central dicemaster.launch.py
```

### ROS2 Introspection Tools

```bash
ros2 node list               # list all running nodes
ros2 topic list              # list all active topics
ros2 topic info /imu/motion  # publishers and subscribers
ros2 topic bw /imu/data      # bandwidth usage
ros2 node info /game_manager # subscriptions, publications, services
ros2 service call /game_control dicemaster_central_msgs/srv/DiceGameControl \
  "{command: 'list', game_name: ''}"   # list available games
```

---

## 8. Common Maintenance Tasks

| Task | Component | Steps | Notes |
|------|-----------|-------|-------|
| Add a new screen command type | Central + ESPScreen | 1. Assign a new `0x0N` byte value in `protocol.py` and `protocol.h`. 2. Add encoding logic in `protocol.py`. 3. Add parsing + rendering in ESP32 firmware. 4. Add the new `media_type` constant to `ScreenMediaCmd` or a new message field. 5. Rebuild Central (`colcon build`), flash all three ESP32s. | Both sides must be updated and deployed atomically. Protocol mismatch causes silent failures. |
| Change SPI baud rate | Central + ESPScreen | 1. Edit `SPIConfig` in `config.py` (Pi side). 2. Edit the SPI clock parameter in the ESP32 Arduino sketch. 3. Rebuild Central (config change triggers rebuild), re-flash ESP32s. | Higher rates may require shorter wiring or signal integrity review on the PCB. |
| Update firmware on all three ESP32s | ESPScreen | 1. Edit firmware in `DiceMaster_ESPScreen/Dicemaster/`. 2. Open Arduino IDE, select the ESP32-S3 board target. 3. Flash each board individually via USB. 4. Verify with serial monitor (115200 baud). | Each ESP32 must be flashed separately. There is no OTA update mechanism. |
| Add a new ROS2 topic | Central | 1. Define a new `.msg` file in `dicemaster_central_msgs/` if a new type is needed. 2. Run `colcon build`. 3. Add publisher in the appropriate node. 4. Add subscriber in the nodes that consume it. 5. Update `docs/architecture/overview.md` topic map. | If reusing an existing message type, skip steps 1-2. Rebuild always required when `.msg` files change. |
| Add a new game (strategy) | Central | 1. Create `~/.dicemaster/games/<name>/` with `config.json` and assets. 2. Create `~/.dicemaster/strategies/<name>/<name>.py` with a `BaseStrategy` subclass. 3. Call `/game_control` service with `command: 'list'` to verify discovery. | No rebuild needed. The game manager discovers strategies via `importlib` scan. |
| Update the dice Python SDK API | Central_Web | 1. Edit the relevant module in `DiceMaster_Central_Web/dice/`. 2. Add/update `postMessage` type handling in `_bridge.py` if the change adds new JS communication. 3. Update `DiceMaster_Studio` to handle the new message type on the JS side. 4. Run `uv run pytest tests/`. | Keep the API surface aligned with `BaseStrategy` in `DiceMaster_Central` for portability. |
| Change screen-to-bus assignment | Central | 1. Edit `dice_config.screen_configs` in `config.py` — update `bus_id` for affected screens. 2. Edit PCB wiring accordingly in KiCad (DiceMaster_HW). 3. Rebuild (`colcon build --symlink-install`), restart system. | Bus IDs 0, 1, 3 are the active buses; bus 2 is reserved. |
| IMU calibration | Central | 1. Run `ros2 service call /imu/calibrate std_srvs/srv/Empty` on the Pi. 2. Update calibration values in `IMUConfig` in `config.py`. | Run calibration with the dice on a flat, still surface. |
| Bump a submodule to latest | Meta-repo | 1. `cd <submodule> && git pull`. 2. `cd .. && git add <submodule> && git commit -m "bump <submodule>"`. | Always update the meta-repo pointer after advancing a submodule. |

---

## 9. Architecture Decision Records Index

ADRs are stored in [docs/decisions/](../decisions/). Each record explains what was decided, why, and what was rejected.

| ADR | Title | One-line Summary |
|-----|-------|-----------------|
| [ADR-001](../decisions/001-ros2-architecture.md) | ROS2 as System Middleware | Use ROS2 Humble pub/sub for all inter-node communication, enabling hot-swappable games and standard IMU tooling at the cost of a source build on Raspberry Pi. |
| [ADR-002](../decisions/002-esp32-spi-screens.md) | ESP32-S3 + SPI for Driving 6 Screens | Offload JPEG decoding, font rendering, and GIF playback to three ESP32-S3 boards over dedicated SPI buses, using CAN-bus bitmask addressing, to avoid saturating the Pi's CPU. |
| [ADR-003](../decisions/003-python-game-api.md) | Strategy Pattern with BaseStrategy | Expose a two-method lifecycle (`start_strategy` / `stop_strategy`) via an abstract base class so educators can write games in plain Python without any knowledge of ROS2 internals. |

---

## 10. Getting Help

**Bug reports and feature requests**: [GitHub Issues](https://github.com/DanielHou315/DiceMaster/issues)

**Design discussions**: [GitHub Discussions](https://github.com/DanielHou315/DiceMaster/discussions)

**Lead developer**: Daniel Hou — [dhou@umich.edu](mailto:dhou@umich.edu)

**Lab**: [U-M Shapiro Design Lab](https://shapirodesignlab.engin.umich.edu/)

### Where to Look for Specific Information

| Question | Where to look |
|---|---|
| "How do I set up a fresh Raspberry Pi?" | [DiceMaster_Central/docs/setup/rpi_setup.md](../../DiceMaster_Central/docs/setup/rpi_setup.md) |
| "How do I deploy a code change?" | [DiceMaster_Central/docs/runbooks/deploy.md](../../DiceMaster_Central/docs/runbooks/deploy.md) |
| "How do I write a game?" | [docs/beginner_game_guide.md](../beginner_game_guide.md) |
| "What are all the ROS2 nodes and topics?" | [docs/architecture/overview.md](../architecture/overview.md) |
| "What is the full SPI protocol spec?" | [docs/protocol.md](../protocol.md) |
| "How does the hardware configuration work?" | [DiceMaster_Central/docs/setup/rpi_hw_config.md](../../DiceMaster_Central/docs/setup/rpi_hw_config.md) |
| "How do I enable auto-start?" | [DiceMaster_Central/docs/setup/auto_start.md](../../DiceMaster_Central/docs/setup/auto_start.md) |
| "What are the screen/IMU APIs?" | [DiceMaster_Central/docs/api/](../../DiceMaster_Central/docs/api/) |
| "Why was ROS2 chosen?" | [docs/decisions/001-ros2-architecture.md](../decisions/001-ros2-architecture.md) |
| "Why three ESP32s over SPI?" | [docs/decisions/002-esp32-spi-screens.md](../decisions/002-esp32-spi-screens.md) |
