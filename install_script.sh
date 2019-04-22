#!/usr/bin/env

cd ../
git clone https://github.com/ccny-ros-pkg/scan_tools
git clone https://github.com/iralabdisco/ira_laser_tools.git

sudo su

apt-get install ros-indigo-robot-upstart
apt-get install ros-indigo-scan-tools
apt-get install ros-indigo-move-base
apt-get install ros-indigo-urg-node
apt-get install ros-indigo-teleop-twist-joy
apt-get install ros-indigo-teleop-twist-keyboard
apt-get install ros-indigo-spacenav-node
apt-get install ros-indigo-yocs-cmd-vel-mux
apt-get install ros-indigo-joy
apt-get install ros-indigo-laser-geometry
apt-get install ros-indigo-laser-scan-matcher
apt-get install libgsl0-dev

sudo -k

cd ../
catkin_make
