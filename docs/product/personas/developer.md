# Persona: Dana Park

## Who They Are

Dana Park is a 31-year-old research software engineer at a university human-computer interaction lab. The lab studies tangible user interfaces, and Dana's current project involves building a set of custom physical learning artifacts for a multi-year funded study on embodied cognition in STEM education. One artifact is a programmable dice-shaped device — which is why Dana is evaluating DiceMaster.

Dana has five years of Python experience and is comfortable with APIs, type systems, and async patterns. Dana has used ROS2 briefly on a previous robotics project and understands the pub/sub model. Dana is also comfortable reading C++ when needed to understand firmware behavior.

Dana's use case goes beyond the default game examples. The research protocol requires:

- Custom motion triggers: not just "is it shaking," but specific rotation gestures (e.g., "flip from face 3 to face 1 within 500ms") detected with configurable thresholds
- Programmatic screen control: sending raw content commands to specific screens at specific times, not just responding to IMU events
- Logging: recording which face was up, when a shake occurred, and what content was displayed, with timestamps, for later analysis
- Integration: a web dashboard on the researcher's laptop that can send commands to the dice over the local network

Dana wants to know exactly what ROS2 topics are published, what the message schemas look like, what the `BaseStrategy` lifecycle guarantees, and where the SPI protocol is implemented so they can understand error recovery behavior. Dana will read source code when documentation is insufficient.

**Technical baseline:** Strong Python, comfortable with ROS2 concepts, reads C++. Will write unit tests. Expects API documentation with type signatures, not just prose descriptions.

## Pain Points

- **Incomplete or missing API docs.** Dana does not want to reverse-engineer `strategy.py` to understand the full `BaseStrategy` interface. Needs documented method signatures, argument types, and lifecycle guarantees.
- **Unclear topic contract.** The full list of ROS2 topics, their message types, and the semantics of each field needs to be documented somewhere authoritative. The architecture diagram in `README.md` gives the shape, but not field-level detail on `MotionDetection` or `ScreenMediaCmd`.
- **Custom motion detection is not exposed.** The `/imu/motion` topic provides coarse-grained shake detection. Dana's gesture recognition needs access to the filtered `/imu/data` stream directly and may need to write a custom motion detector node. It is not immediately clear whether this is supported or how to slot it into the existing launch graph.
- **Protocol edge cases are underdocumented.** The SPI chunked transfer protocol has timeout-based invalidation. Dana needs to understand what happens when a chunk is dropped, whether the `ScreenBusManager` retries, and how to detect that a screen is out of sync.
- **Build complexity.** The custom `py-spidev` build and the ROS2 workspace setup are not scripted end-to-end. Dana wants a reproducible setup that can be versioned and re-deployed on a replacement Pi with minimal manual steps.

## Journey

**Evaluation.** Dana reads the DiceMaster README and architecture diagram. Immediately opens `DiceMaster_Central/docs/api/architecture.md` to understand the ROS2 node graph. Checks the `dicemaster_central_msgs` package to read the raw message definitions. Runs `ros2 topic list` and `ros2 interface show` on a running system to verify the documented topics match reality.

**First custom strategy.** Dana clones the repository, sets up the dev environment per `docs/setup/dev_setup.md`, and writes a strategy that subscribes directly to both `/imu/motion` and `/imu/data`. Tests that the gesture classification code works on recorded IMU playback before connecting to the live device.

**Custom motion detector.** Dana forks `motion_detector.py` to implement the gesture recognition algorithm the research protocol requires. Adds it to the launch file as a peer node alongside the existing `motion_detector`. Both publish to different topic names, and the game strategy subscribes to the custom topic.

**Logging integration.** Dana adds a separate ROS2 node that subscribes to `/imu/motion`, `/chassis/orientation`, and all `/screen_{id}_cmd` topics and writes timestamped records to a CSV file. This node is added to `managers.launch.py` as an optional component.

**Web dashboard.** Dana implements a small FastAPI server that wraps the `/game_control` ROS2 service, allowing the researcher's laptop to switch games and trigger screen commands over HTTP. The server runs as a background process on the Pi alongside the main DiceMaster launch.

**Filing issues.** Dana files two GitHub issues: one requesting that the full `MotionDetection` message field semantics be documented (specifically the units of `rotation_intensity` and `shake_intensity`), and one noting that the chunked transfer timeout behavior is only described in the protocol spec but not in the Python `ScreenBusManager` source.

## Content Needs

- **Full ROS2 topic reference** with message type, field-level documentation, units, and publishing frequency for every topic in the system
- **BaseStrategy API reference** with complete method signatures, lifecycle sequence diagram, and notes on which methods are safe to call from a timer callback vs. a subscription callback
- **ScreenMediaCmd reference** documenting all `media_type` values, the expected `file_path` format, and how priority affects the display queue
- **Protocol specification** (already exists at `docs/protocol.md`) with a note on retry and error recovery behavior at the `ScreenBusManager` level
- **Architecture deep-dive** covering the `MultiThreadedExecutor` configuration, how the game manager loads and unloads strategy nodes, and thread safety guarantees
- **Developer setup guide** with a fully scripted path from a fresh Raspberry Pi OS install to a running DiceMaster system, ideally idempotent
- **Example of a custom launch file extension** showing how to add a peer node (e.g., a custom motion detector or logger) without modifying the core launch files
