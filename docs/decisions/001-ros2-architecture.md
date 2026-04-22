# ADR-001: ROS2 as System Middleware

## Status

Accepted

## Context

DiceMaster requires coordinated communication between several independent subsystems running on a single Raspberry Pi 4:

- A game logic layer (user-written Python strategies) that reacts to sensor events
- An IMU pipeline that reads raw sensor data from an MPU6050 over I2C, filters it with the Madgwick algorithm, and publishes shake/orientation detections
- A screen layer consisting of 3 SPI bus manager nodes, each driving 2 ESP32-backed displays

These subsystems need to exchange data continuously at runtime without tight coupling. The game logic must be hot-swappable (i.e., the active game can be changed without restarting the IMU pipeline or screen managers). Sensor data must be fanned out to both the chassis orientation tracker and the motion detector simultaneously. A beginner-level Python API for educators writing games must not require them to understand inter-process communication.

The system was also designed with research extensibility in mind: future versions may run perception nodes, log data, or visualize state, all of which benefit from a standardized message bus.

## Decision

Use **ROS2 Humble** as the middleware. Each major subsystem runs as a ROS2 node. Data flows through typed topic messages on a shared DDS pub/sub bus. The full node graph is:

```
imu_hardware → /imu/data_raw → imu_filter_madgwick → /imu/data
                                    ├→ motion_detector → /imu/motion
                                    └→ chassis          → /chassis/orientation
                                                          /chassis/screen_{id}_pose
game strategy   → /screen_{id}_cmd → screen_bus_manager → SPI → ESP32
```

Custom message types (`MotionDetection`, `ChassisOrientation`, `ScreenPose`, `ScreenMediaCmd`) are defined in the `dicemaster_central_msgs` package and compiled at build time.

The game manager node uses a `MultiThreadedExecutor` to host the active strategy node as a dynamically loaded child, allowing game swaps via the `/game_control` ROS2 service without tearing down the rest of the graph.

## Consequences

**Positive:**
- Nodes are decoupled by topic interfaces. The IMU pipeline, chassis tracker, screen managers, and game logic can all be developed and tested independently.
- The Madgwick IMU filter (`imu_filter_madgwick`) is a mature third-party ROS2 package that required no custom implementation.
- `colcon build --symlink-install` means Python source edits are reflected immediately without a rebuild cycle.
- Standard ROS2 tooling (`ros2 topic echo`, `ros2 node list`, `rqt_graph`) makes runtime debugging straightforward.
- Future nodes (data logging, web visualization, external control) can subscribe to existing topics with zero changes to existing code.

**Negative:**
- ROS2 Humble must be built from source on Raspberry Pi (aarch64/Debian Bookworm), which is a multi-hour one-time build step.
- Contributors unfamiliar with ROS2 face a learning curve before they can navigate the codebase.
- The `dicemaster_central_msgs` package requires a `colcon build` whenever message definitions change.
- DDS adds latency overhead that would be absent in a direct Python function-call architecture, though this is imperceptible at DiceMaster's update rates.

## Alternatives Considered

**Raw Python threads with shared queues**
Would have been simpler to set up but would have required hand-rolling all the pub/sub infrastructure, message serialization, and lifecycle management that ROS2 provides. Hot-swapping games would have required careful thread management. Debugging would have been harder without standard introspection tools.

**ZeroMQ (ZMQ)**
ZMQ provides a lightweight pub/sub transport without the ROS2 build complexity. However, it lacks the typed message schema system, standard IMU filter ecosystem, and the broader ROS2 tooling. Integrating `imu_filter_madgwick` would have required a custom bridge. ZMQ was judged to provide insufficient value over ROS2 given the existing ROS2 ecosystem investment.
