# Feature TODO

## USB Mass Storage Emulation

Emulate a USB mass storage device over USB with a fixed 2GB storage partition. Allows users to drag-and-drop game assets onto the dice without SSH or network access. The DiceMaster program must not access the storage partition while USB is connected to a host.

### Investigation notes (2026-03-08)

**Software: ready.** All kernel modules (`g_mass_storage`, `libcomposite`, `dwc2`) load correctly. A 2GB FAT32 image and systemd service are configured. DWC2 registers confirm correct peripheral mode configuration.

**Blocker: Waveshare CM4-NANO-B lacks USB mux for gadget mode.** Confirmed by testing and corroborated by [RPi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=347459) where nobody got gadget mode working on CM4-NANO-A/B boards. The original poster confirmed it only worked on the official CM4 IO Board, which has a USB2514B-I/M2 hub controller and a dedicated USB mux chip for role switching.

On the CM4-NANO-B:
- DWC2 in host mode drives the USB-A port (confirmed working)
- DWC2 in peripheral mode: registers configure correctly but the physical USB-A port lacks the mux circuitry to signal device mode to the host. The port is hardwired as host-only.
- USB-C port is shared with power input and uses DWC2 OTG lines (used by BOOT mode for eMMC flashing)

**Resolution options:**
1. **Power via PiSugar battery or GPIO 5V** → free USB-C for gadget data. Software is ready.
2. **USB-C hub with power delivery pass-through** → power + data over one port.
3. **Switch to official CM4 IO Board** or a carrier board with a dedicated OTG port and USB mux.
4. **HW v2 PCB:** Include a USB mux (like the CM4 IO Board does) to support gadget mode on a dedicated connector.

**Current Pi config (`dice1`):** Reverted to `dtoverlay=dwc2,dr_mode=host` (Waveshare recommended). Artifacts remaining: `/opt/usb_storage.img` (2GB FAT32), `/etc/systemd/system/usb-gadget.service` (disabled).

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
