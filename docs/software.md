# Programmable Dice Software Reference

# Supported Languages
Supported languages is limited by supported fonts in the U8g2 library for Arduino and how much space is available 

# Developing Software

## For Screen Modules

**Factory Reset**

- It is inevitable that you will upload some program that cause segmentation faults. In this case, the board hard-crashes and is unable to receive your further uploads through the USB port. To factory-reset the board and erase the existing (buggy) program, drag over the factory default UF2 file located in the `resources/` directory. 

**Dependencies**
- esp32 (2.0.17)
- 
- ESP32DMASPI (0.6.5)
- GFX Library for Arduino (1.4.9)
- JPEGDEC (1.8.2)
- U8g2 (2.35.30)

## For Raspberry Pi
- Flash Ubuntu 22.04 arm64 image
	- Make sure to configure username and pwd in pi-imager
- (Dev only) Setup SSH
	- Configure SSH 
- [Install ROS2 humble via apt](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources)
- Get the DiceMaster_Central package from github
- Compile package with `colcon build`
- Configure auto-start of package entrypoint.

## For User Computer GUI
- (Dev only) configure github access
- (Dev only) clone repo and start working
- Download the respective executables from github (TODO: build CI workflow for auto-build)
