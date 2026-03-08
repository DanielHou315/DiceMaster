# Feature TODO

## USB Mass Storage Emulation

Emulate a USB mass storage device over the USB-A port with a fixed 2GB storage partition. Allows users to drag-and-drop game assets onto the dice without SSH or network access.

## System Resource Reduction

Strip the OS and runtime environment to the minimal set needed for operation. Remove unnecessary services, packages, and kernel modules beyond what is useful for development or runtime.

## Hot Reload for Games

Support reloading games without restarting the full ROS2 stack. Detect changes in game configs/assets and re-instantiate the relevant strategy node on the fly.

## Battery Estimation

Read battery voltage/current and provide runtime estimates. Publish battery state to a ROS2 topic for use by games or system UI (e.g. low-battery warnings on screens).

## GPIO Button Input

Support physical button presses via GPIO pins. Publish button events to ROS2 topics for use by game strategies or system control (e.g. game selection, power control).

## Screen Sleep

Put ESP32 screens into a low-power or blanked state after inactivity or on command. Wake on motion (IMU) or button press.

## Automated Fan Control

Control a cooling fan via GPIO based on system temperature readings. Implement threshold-based or proportional speed control to manage thermals quietly.

## Node Profiling & Event-Driven Optimization

Profile existing ROS2 nodes for CPU usage and communication overhead. Replace polling and timer-based patterns with event-driven approaches where possible to reduce idle CPU load and unnecessary message traffic.

## C++ Consolidation

Profile performance-critical or repeated Python paths (e.g. SPI communication, protocol encoding) and consolidate into C++ ROS2 nodes or libraries where the overhead justifies it.
