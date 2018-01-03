# Raspberry Pi 3 setup for Opticar ADAS

## Prerequisites
1. Raspberry Pi 3
2. (optional) SK-Pang PiCAN 2 Hat, assembled on top of the Pi
3. MicroSD-Card 32GB

## System setup
1. Download Ubuntu MATE 16.04 LTS for Raspberry Pi from https://ubuntu-mate.org/download/#xenial
2. Flash it to the SD card. When using Win32DiskImager on Windows 10, you might have to remove all existing partitions prior to flashing.
3. Insert the card into the Pi
4. Connect keyboard, mouse, HDMI, network and power
5. Let the system boot and follow the system configuration wizard
  * Hostname `opticar-adas`
  * User name `opticar`, password `changeme`
6. Using `raspi-config`, enable SSH
7. Now you can use e.g. SmarTTY to connect to the Pi
8. Update the system
   ```bash
   sudo apt update
   sudo apt dist-upgrade
   ```

## ROS installation
1. Follow the guide at http://wiki.ros.org/kinetic/Installation/Ubuntu
2. Also install additional packages
   ```bash
   sudo apt install ros-kinetic-rosserial-server ros-kinetic-rosserial-tivac python-catkin-tools ros-kinetic-imu-filter-madgwick ros-kinetic-gmapping ros-kinetic-map-server ros-kinetic-navigation ros-kinetic-joy ros-kinetic-rosbridge-suite ros-kinetic-teleop-twist-joy
   ```
3. Install the embedded ARM toolchain and make sure you can connect to the Tiva board as user
   ```bash
   sudo apt-get install gcc-arm-none-eabi

   git clone https://github.com/utzig/lm4tools.git
   cd lm4tools/lm4flash
   make
   sudo cp lm4flash /usr/local/bin

   echo 'ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", GROUP="plugdev", MODE="0666"' | sudo tee /etc/udev/rules.d/99-stellaris-launchpad.rules
   ```
4. Install Tivaware. Example below:
   ```bash
   mkdir <TivaWarePah>
   cd <TivaWarePah>
   mv <directory_downloaded>/SW-TM4C-2.1.1.71.exe
   unzip SW-TM4C-2.1.1.71.exe
   rm SW-TM4C-2.1.1.71.exe
   ```
5. Create catkin workspace
      ```bash
      mkdir -p ~/catkin_ws/src
      cd ~/catkin_ws/
      catkin_make
      ```
6. Setup environment variables in .bashrc
   ```bash
   source $HOME/catkin_ws/devel/setup.bash
   export TIVA_WARE_PATH=$HOME/<path_to_tivaware_root>
   export TIVA_FLASH_EXECUTABLE=lm4flash
   ```

## WiFi access point Installation
1. Follow the guide at https://frillip.com/using-your-raspberry-pi-3-as-a-wifi-access-point-with-hostapd/ with the following changes:
   * Mate Linux does not used dhcpcd, so just set the wifi interface address to 172.30.0.20/16
     ```bash
     allow-hotplug wlan0
     iface wlan0 inet static
       address 172.30.0.20
       netmask 255.255.0.0
       network 172.30.0.0
       broadcast 172.30.255.255
     ```
   * Use this for hostapd.conf
     ```bash
     # This is the name of the WiFi interface we configured above
     interface=wlan0

     # Use the nl80211 driver with the brcmfmac driver
     driver=nl80211

     # This is the name of the network
     ssid=changeme

     # Use the 2.4GHz band
     hw_mode=g

     # Use channel 6
     channel=6

     # Enable 802.11n
     ieee80211n=1

     # Enable WMM
     wmm_enabled=1

     # Enable 40MHz channels with 20ns guard interval
     ht_capab=[HT40][SHORT-GI-20][DSSS_CCK-40]

     # Accept all MAC addresses
     macaddr_acl=0

     # Use WPA authentication
     auth_algs=1

     # Require clients to know the network name
     ignore_broadcast_ssid=0

     # Use WPA2
     wpa=2

     # Use a pre-shared key
     wpa_key_mgmt=WPA-PSK

     # The network passphrase
     wpa_passphrase=changeme

     # Use AES, instead of TKIP
     rsn_pairwise=CCMP
     ```
   * Use this for dnsmasq.conf
     ```bash
     interface=wlan0      # Use interface wlan0  
     listen-address=172.30.0.20 # Explicitly specify the address to listen on  
     bind-interfaces      # Bind to the interface to make sure we aren't sending things  elsewhere  
     server=8.8.8.8       # Forward DNS requests to Google DNS  
     domain-needed        # Don't forward short names  
     bogus-priv           # Never forward addresses in the non-routed address spaces.  
     dhcp-range=172.30.0.100,172.30.0.150,12h # Assign IP addresses between 172.24.1.50 and 172.24.1.150 with a 12 hour lease time
     ```
  * When setting up IPv4 forwarding, take note of the changed network interface name for the ethernet interface

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
1. `wstool set opticar_msgs --git https://github.com/opticar/opticar_msgs.git`
1. `wstool set opticar_base --git https://github.com/opticar/opticar_base.git`
1. `wstool update`
1. Flash the software to the embedded board:
   ```bash
   catkin build opticar_base
   catkin build --no-deps opticar_base --make-args opticar_base_tiva_flash
   ```
1. Create a `58-opticar.rules` file in `/etc/udev/rules.d/` with
   ```bash
   KERNEL=="ttyACM?", SUBSYSTEM=="tty", ATTRS{idVendor}=="1cbe", ATTRS{idProduct}=="00fd", MODE="0666" SYMLINK+="tiva"
   KERNEL=="js[0-9]*", ENV{ID_BUS}=="?*", ENV{ID_INPUT_JOYSTICK}=="?*", GROUP="input", MODE="0664"
   ```

## ROS testing
1. One terminal running `roscore`
2. One terminal running `rosrun rosserial_python serial_node.py _port:=/dev/tiva _baud:=115200`
3. One terminal running `roslaunch rosbridge_server rosbridge_websocket.launch`

## ROS autostart
1. In order to have a launch file (e.g. the teleop configuration) executed at startup, run
   ```bash
   rosrun robot_upstart install --user opticar opticar_base/launch/teleop.launch
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
