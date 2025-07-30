# Programmable Dice Software Reference

# Supported Languages
Supported languages is limited by supported fonts in the U8g2 library for Arduino and how much space is available 

# Developing Software

## For Screen Modules

**Factory Reset**

- It is inevitable that you will upload some program that cause segmentation faults. In this case, the board hard-crashes and is unable to receive your further uploads through the USB port. To factory-reset the board and erase the existing (buggy) program, drag over the factory default UF2 file located in the `resources/` directory. 

**Dependencies**
- esp32 (2.0.17)
- ESP32DMASPI (0.6.5)
- GFX Library for Arduino (1.4.9)
- JPEGDEC (1.8.2)
- U8g2 (2.35.30)

## For Raspberry Pi
1. Flash Raspbian OS 64-bit (full for GUI desktop experience, lite for commandline-only)
	- Make sure to configure username and passwords in pi-imager
2. (Dev only) Setup SSH in pi-imager
	- Configure SSH keys from your device
	- (Optional) Install zerotier for easier access through private VPN. 
3. Install, update packages, and configure OS
```bash
sudo apt update
sudo apt upgrade
sudo apt install git
```
4. Configure os with
```bash
sudo raspi-config
```
and in `interface options` enable SSH, SPI, and I2C protocols. 

5. Configure SPI and I2C interfaces by following `DiceMaster_Central/docs/HW_interfaces.md`. 

6. Install ROS. Since Debian is tier-3 supported by ROS2, you need to compile from source
- [Install ROS2 humble via apt](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources)

7.  Get the DiceMaster_Central package from github and compile for ROS
	You may want to use the DiceMaster_ROS_Workspace for some convenient ROS environment setup and packaging. 
```bash
git clone git@github.com:DanielHou315/DiceMaster.git --recursive
cd DiceMaster/DiceMaster_ROS_Workspace
mkdir -p src && ln -s ../DiceMaster_Central .
source prepare.sh
colcon build --symlink-install
```

9. Configure auto-start of package entrypoint.

## For User Computer App
- (Dev only) configure github access
- (Dev only) clone repo and start working
- Download the respective executables from github (TODO: build CI workflow for auto-build)
