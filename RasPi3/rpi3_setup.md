# Raspberry Pi 3 setup for Opticar ADAS

## Prerequisites
1. Raspberry Pi 3
1. (optional) SK-Pang PiCAN 2 Hat, assembled on top of the Pi
1. MicroSD-Card 32GB

## System setup
1. Download Ubuntu 18.04 for armhf (e.g. http://cdimage.ubuntu.com/releases/bionic/release/ubuntu-18.04.3-preinstalled-server-armhf+raspi3.img.xz)
1. Flash it to the SD card. When using Win32DiskImager on Windows 10, you might have to remove all existing partitions prior to flashing.
1. Insert the card into the Pi
1. Connect keyboard, mouse, HDMI, network and power
1. Let the system boot and log in as user `ubuntu` with password `ubuntu`
1. You will be forced to change the password, so do that
1. Change your keyboard layout if needed: `sudo dpkg-reconfigure keyboard-configuration`
1. Install the Z shell: `sudo apt install zsh`
1. Add the opticar user:
   ```bash
   sudo useradd -m -s /bin/zsh -G adm,dialout,cdrom,floppy,sudo,audio,dip,video,plugdev,lxd,netdev opticar
   ```
1. Set the password to `opticar`using `sudo passwd opticar`
1. Log out and log back in as user `opticar`
1. Setup your shell if prompted
1. Set the hostname: `sudo hostnamectl set-hostname opticar-adas`
1. Update the system
   ```bash
   sudo apt update
   sudo apt dist-upgrade
   ```

## ROS installation
1. Follow the guide at http://wiki.ros.org/melodic/Installation/Ubuntu, Steps 1.1 to 1.7 (desktop installation is optional)
1. Also install additional packages
   ```bash
   sudo apt install ros-melodic-rosserial-server ros-melodic-rosserial-tivac python-catkin-tools ros-melodic-imu-filter-madgwick ros-melodic-gmapping ros-melodic-map-server ros-melodic-navigation ros-melodic-joy ros-melodic-rosbridge-suite ros-melodic-teleop-twist-joy dphys-swapfile ros-melodic-cv-bridge ros-melodic-image-transport ros-melodic-robot-upstart
   ```
1. Install the embedded ARM toolchain and make sure you can connect to the Tiva board as user
   ```bash
   sudo apt-get install gcc-arm-none-eabi libusb-1.0-0-dev unzip

   git clone https://github.com/utzig/lm4tools.git
   cd lm4tools/lm4flash
   make
   sudo cp lm4flash /usr/local/bin

   echo 'ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", GROUP="plugdev", MODE="0666"' | sudo tee /etc/udev/rules.d/99-stellaris-launchpad.rules
   ```
1. Downlaod and install Tivaware. Example below:
   ```bash
   cd $HOME
   mkdir TivaWare
   cd TivaWare
   mv <directory_downloaded>/SW-TM4C-2.1.1.71.exe
   unzip SW-TM4C-2.1.1.71.exe
   rm SW-TM4C-2.1.1.71.exe
   ```
1. Create catkin workspace
      ```bash
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws/
      catkin build
      ```
1. Setup environment variables in .zshrc
   ```bash
   echo 'source $HOME/catkin_ws/devel/setup.zsh' | tee -a ~/.zshrc
   echo 'export TIVA_WARE_PATH=$HOME/TivaWare' | tee -a ~/.zshrc
   echo 'export TIVA_FLASH_EXECUTABLE=lm4flash' | tee -a ~/.zshrc
   ```
1. Use the updated configuration: `source ~/.zshrc`

## WiFi access point Installation
1. Install the corresponding snap: `snap install wifi-ap`
1. Configure the access point: `wifi-ap.config set wifi.security=wpa2 wifi.security-passphrase=opticaradas wifi.address=172.30.0.20 wifi.netmask=255.255.0.0 wifi.ssid=opticar dhcp.range-start=172.30.0.100 dhcp.range-stop=172.30.0.199 wifi.country-code=DE`
1. Restart the access point: `wifi-ap.status restart-ap`

## CAN bus setup (optional)
* Make sure the PiCAN2 hat is installed on the RPi
* Add the following lines at the end of /boot/config.txt
  ```bash
  dtparam=spi=on
  dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=25
  ```
* Add the following lines to /etc/network/interfaces
  ```bash
  auto can0
  iface can0 inet manual
    pre-up /sbin/ip link set $IFACE type can bitrate 500000 triple-sampling on
    up /sbin/ifconfig $IFACE up
    down /sbin/ifconfig $IFACE down
  ```
* Install utilities: `sudo apt install can-utils`

## ROS software setup
1. `cd ~/catkin_ws/src`
1. `wstool init`
1. `wstool set opticar_msgs --git https://github.com/opticar/opticar_msgs.git`
1. `wstool set opticar_base --git https://github.com/opticar/opticar_base.git`
1. `wstool update`
1. Create a `58-opticar.rules` file in `/etc/udev/rules.d/` with
   ```bash
   KERNEL=="ttyACM?", SUBSYSTEM=="tty", ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", MODE="0666" SYMLINK+="tiva"
   KERNEL=="js[0-9]*", ENV{ID_BUS}=="?*", ENV{ID_INPUT_JOYSTICK}=="?*", GROUP="input", MODE="0664"
   ```
1. Reload udev rules: `sudo udevadm control --reload-rules && sudo udevadm trigger`
1. Flash the software to the embedded board:
   ```bash
   catkin build opticar_base
   catkin build --no-deps opticar_base --make-args opticar_base_tiva_flash
   ```

## ROS testing
1. One terminal running `roscore`
1. One terminal running `rosrun rosserial_python serial_node.py _port:=/dev/tiva _baud:=115200`
1. One terminal running `roslaunch rosbridge_server rosbridge_websocket.launch`

## ROS autostart
1. In order to have a launch file (e.g. the teleop configuration) executed at startup, run
   ```bash
   rosrun robot_upstart install --user opticar opticar_base/launch/teleop.launch
   ```
1. To remove this autostart job, run
   ```bash
   rosrun robot_upstart uninstall opticar
   ```

## ROS joystick
1. The elements of an XBox 360 Controller are as follows:
   * Left analog stick: horizontal axis 0, vertical axis 1
   * Left shoulder: axis 2
   * Right analog stick: horizontal axis 3, vertical axis 4
   * Right shoulder: axis 5
   * Direction pad: horizontal axis 6, vertical axis 7
   * Buttons: A:0, B:1, X:2, Y:3, LT: 4, RT: 5, Back:6, Start:7, XBox: 8, Left analog: 10, Right analog: 11, Pad L: 12, Pad R: 13, Pad U: 14, Pad D: 15

## Todo
* IMU
* GPS
* socketcan
* rep 103
* rep 105
* robot_localization
* parameter server
