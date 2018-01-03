# Host PC setup for Opticar ADAS

## Prerequisites
1. Ubuntu 16.04 or Windows 10 1703 with WSL
2. ROS kinetic with wstool and catkin_tools

## ROS software setup
* `sudo apt install ros-kinetic-teleop-twist-keyboard`
* `cd your_catkin_ws/src`
* `wstool set opticar_msgs --git https://github.com/opticar/opticar_msgs.git`
* `wstool set opticar_base --git https://github.com/opticar/opticar_base.git`
* `wstool update`
* `cd .. && catkin_make`
