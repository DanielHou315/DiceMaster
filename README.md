<div align="center">

# 🎲 DiceMaster

### *A Programmable Multi-Screen Dice for Interactive Learning*

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2: Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Platform: Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry%20Pi-red.svg)](https://www.raspberrypi.org/)
[![Alpha Release](https://img.shields.io/badge/Status-Alpha-orange.svg)](https://github.com/DanielHou315/DiceMaster)

[Features](#-features) • [How It Works](#-how-it-works) • [Getting Started](#-getting-started) • [Documentation](#-documentation) • [Contributing](#-contributing)

---

</div>

## 🌟 About

**DiceMaster** is a revolutionary educational device that combines physical interaction with digital learning. Built with six high-resolution screens embedded in a dice form factor, it creates engaging, hands-free learning experiences through motion-based interaction.

Originally designed for language learning, DiceMaster detects shaking, rotation, and orientation to trigger dynamic content changes across all screens. Students can learn vocabulary, grammar, and more through interactive flashcards, quizzes, and games—all without touching a single button.

### 🎯 Presented at SeriousPlay 2025

DiceMaster was showcased at the **SeriousPlay 2025** conference, demonstrating its potential for game-based learning and pedagogical innovation. Stay tuned for research findings and student feedback!

---

## ✨ Features

### 🖥️ **Six 480×480 Displays**
- **ESP32-powered screens** with full-color JPEG support
- Animated GIF playback capability
- Multi-language text rendering (Arabic, Chinese, Cyrillic, Devanagari, and more)
- Dynamic orientation-aware content display

### 🧠 **Motion-Aware Intelligence**
- **IMU-based motion detection** (MPU6050 6-axis sensor)
- Shake detection with configurable sensitivity
- Automatic orientation detection (which side is facing up/down)
- Screen rotation compensation for optimal readability

### 🎮 **Programmable Game System**
- **ROS2-based architecture** for flexible game development
- Create custom games with simple Python classes
- Asset-based design: JSON text files, images, and animated GIFs
- Hot-swappable games without system restart

### 🔌 **Robust Communication**
- **Custom SPI protocol** with CAN-bus-like addressing
- Chunked image transmission for large media files
- Priority-based message queue for responsive UI
- DMA-optimized data transfer for performance

### 🛠️ **Developer-Friendly**
- Comprehensive Python API built on ROS2
- Extensive documentation for beginners and maintainers
- Example games and strategies included
- Modular architecture for easy extension

---

## 🔧 How It Works

```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi Central                     │
│                                                              │
│  ┌──────────┐    ┌───────────┐    ┌──────────────┐        │
│  │   Game   │───▶│  Chassis  │◀───│  IMU Sensor  │        │
│  │ Strategy │    │  Manager  │    │  (Motion)    │        │
│  └────┬─────┘    └─────┬─────┘    └──────────────┘        │
│       │                │                                     │
│       │ Screen Commands│                                     │
│       ▼                ▼                                     │
│  ┌──────────────────────────────────────┐                  │
│  │   Screen Bus Managers (SPI)          │                  │
│  └──────────────┬───────────────────────┘                  │
└─────────────────┼──────────────────────────────────────────┘
                  │
       ┌──────────┼──────────┐
       ▼          ▼          ▼
  ┌────────┐ ┌────────┐ ┌────────┐
  │ ESP32  │ │ ESP32  │ │ ESP32  │
  │ Screen │ │ Screen │ │ Screen │
  │  1 & 2 │ │  3 & 4 │ │  5 & 6 │
  └────────┘ └────────┘ └────────┘
```

### The Magic Behind DiceMaster:

1. **Sensors Detect Motion**: IMU tracks shaking, rotation, and orientation
2. **Games React**: Your Python strategy responds to sensor events
3. **Content Updates**: Text, images, or GIFs are sent to specific screens
4. **Screens Display**: ESP32 modules render content with proper rotation
5. **Learning Happens**: Students interact naturally through physical manipulation

---

## 🚀 Getting Started

### 📚 For Game Creators

Want to create your first educational game? Check out our comprehensive guide:

**👉 [Beginner's Game Development Guide](docs/beginner_game_guide.md)**

Learn how to:
- Create a "Hello Dice" game in 30 minutes
- Use text, images, and animated GIFs
- Respond to shaking and rotation
- Build interactive quizzes and flashcards

### Quick Example

```python
from dicemaster_central.games.strategy import BaseStrategy
from dicemaster_central_msgs.msg import MotionDetection

class MyGame(BaseStrategy):
    _strategy_name = "my_game"

    def start_strategy(self):
        # Subscribe to shake detection
        self.create_subscription(
            MotionDetection, '/imu/motion',
            self.on_shake, 10
        )
        # Display content on all screens
        self.display_greeting()

    def on_shake(self, msg):
        if msg.shaking:
            # Change content when shaken!
            self.display_next_question()
```

---

## 📖 Documentation

### 📘 **Comprehensive Guides**

| Guide | Description | Audience |
|-------|-------------|----------|
| [**Beginner's Guide**](docs/beginner_game_guide.md) | Step-by-step tutorial for creating your first game | Game Developers |
| [**Developer Guide**](docs/developer_guide.md) | In-depth system architecture and maintenance | System Developers |
| [**CLAUDE.md**](CLAUDE.md) | Quick reference for AI-assisted development | All Developers |
| [**Protocol Specification**](docs/protocol.md) | Complete SPI communication protocol | Hardware/Firmware Developers |
| [**Hardware Guide**](docs/hardware.md) | Assembly and hardware configuration | Hardware Engineers |
| [**Software Setup**](docs/software.md) | Installation and deployment instructions | DevOps/Administrators |

### 🗂️ **Repository Structure**

```
DiceMaster/
├── DiceMaster_Central/           # Raspberry Pi ROS2 codebase
│   ├── dicemaster_central/       # Main Python package
│   │   ├── games/                # Game system core
│   │   ├── hw/                   # Hardware interfaces (SPI, I2C, IMU)
│   │   ├── managers/             # Game manager and lifecycle
│   │   └── media_typing/         # Protocol implementation
│   ├── examples/                 # Example games and strategies
│   │   ├── games/
│   │   │   └── chinese_quizlet/  # Language learning example
│   │   └── strategies/
│   │       └── shake_quizlet/    # Motion-based quiz strategy
│   └── launch/                   # ROS2 launch files
├── DiceMaster_ESPScreen/         # ESP32 Arduino firmware
├── DiceMaster_ROS_workspace/     # ROS2 build workspace
└── docs/                         # Documentation
```

---

## 🛠️ Building Your Own DiceMaster

### 🏗️ Hardware Requirements

- **Raspberry Pi 4** (or Compute Module 4)
- **6× ESP32-S3 boards** with 480×480 IPS displays
- **MPU6050** 6-axis IMU sensor
- Custom 3D-printed dice enclosure
- Power supply and wiring harness

### 📦 Software Stack

- **ROS2 Humble** (built from source for Raspberry Pi)
- **Python 3.10+** with custom packages
- **Arduino IDE** for ESP32 firmware
- Custom `py-spidev` library (8KB buffer support)

### 🔨 Build Instructions

**Full assembly and build instructions coming soon!**

We're finalizing documentation for:
- 3D printing the enclosure
- PCB assembly and wiring
- Raspberry Pi configuration
- ESP32 firmware flashing
- Complete system integration

Want to be notified? Star this repo and watch for updates! ⭐

---

## 🎮 Example Games

### Chinese Quizlet

A language learning game that displays vocabulary questions with image hints. Shake to cycle through questions!

- **Top Screen**: Question text ("What is this in Chinese?")
- **Bottom Screen**: Answer (e.g., "猫" - cat)
- **Side Screens**: Four hint images
- **Interaction**: Shake to change question

### Pipeline Test

A diagnostic tool that cycles through all screens with test messages.

- Verifies all screens are working
- Tests SPI communication
- Demonstrates screen addressing

---

## 🤝 Contributing

We welcome contributions from the community! Whether you're building new games, improving documentation, or enhancing the core system, your help is appreciated.

### Ways to Contribute

- 🎮 **Create Games**: Share your educational game strategies
- 📝 **Improve Docs**: Help make our guides clearer
- 🐛 **Report Issues**: Found a bug? Let us know
- 💡 **Suggest Features**: Have ideas? We're listening
- 🔧 **Submit PRs**: Code contributions welcome

### Development Setup

```bash
# Clone with submodules
git clone --recursive https://github.com/DanielHou315/DiceMaster.git

# Build ROS2 workspace
cd DiceMaster/DiceMaster_ROS_workspace
source prepare.sh
colcon build --symlink-install

# Run tests
cd ../DiceMaster_Central/dicemaster_central
python3 -m pytest tests/
```

---

## 🙏 Acknowledgments

### Contributors

**DiceMaster Team** - University of Michigan Shapiro Design Lab
- Lead Developer: Daniel Hou

### Built With

DiceMaster stands on the shoulders of amazing open-source projects:

#### Core Technologies
- [**ROS2 Humble**](https://docs.ros.org/en/humble/) - Robot Operating System 2
- [**imu_tools**](https://github.com/CCNYRoboticsLab/imu_tools) - IMU filtering and orientation estimation
- [**Madgwick Filter**](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/) - Orientation algorithm by Sebastian Madgwick

#### Hardware Interfaces
- [**py-spidev**](https://github.com/doceme/py-spidev) - Python SPI interface (custom build)
- [**smbus2**](https://github.com/kplindegaard/smbus2) - Pure Python I2C library

#### ESP32 Development
- [**Arduino-ESP32**](https://github.com/espressif/arduino-esp32) - ESP32 Arduino core
- [**GFX Library for Arduino**](https://github.com/moononournation/Arduino_GFX) - High-performance display library
- [**U8g2**](https://github.com/olikraus/u8g2) - Universal graphics library with multi-language font support
- [**JPEGDEC**](https://github.com/bitbank2/JPEGDEC) - Optimized JPEG decoder

#### Python Ecosystem
- [**Pydantic**](https://pydantic-docs.helpmanual.io/) - Data validation
- [**Pillow**](https://python-pillow.org/) - Image processing
- [**NumPy**](https://numpy.org/) - Numerical computing

### Special Thanks

- **U-M Shapiro Design Lab** for supporting this research project
- **SeriousPlay 2025** conference for providing a platform to share our work
- The **ROS2 community** for incredible tools and support
- All educators who provided feedback during alpha testing

---

## 📄 License

This project is licensed under the **MIT License** - see the [LICENSE](LICENSE) file for details.

You are free to use, modify, and distribute DiceMaster for educational and commercial purposes.

---

## 📬 Contact & Support

- 🐛 **Issues**: [GitHub Issues](https://github.com/DanielHou315/DiceMaster/issues)
- 💬 **Discussions**: [GitHub Discussions](https://github.com/DanielHou315/DiceMaster/discussions)
- 📧 **Email**: [Daniel Hou](mailto:dhou@umich.edu)
- 🏢 **Lab**: [U-M Shapiro Design Lab](https://shapirodesignlab.engin.umich.edu/)

---

<div align="center">

**Made with ❤️ by the Shapiro Design Lab at the University of Michigan**

*Empowering education through interactive technology*

[⬆ Back to Top](#-dicemaster)

</div>