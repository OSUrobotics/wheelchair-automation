wheelchair-automation
=====================

Code for automating a PERMOBIL wheelchair for the ALS foundation.

Package Descriptions
---------------------

wheelchair_description: URDF and launch files for wheelchair initial bringup.

wheelchair_navigation: AMCL and Navigation parameters for the wheelchair.

wheelchair_bringup: Nothing ATM

wheelchair_mapping: Basic gmapping launch file and where maps should be saved.

Useful launch files
---------------------

Gazebo (Only for simulation): bash gazebo_spawn_swivel_willow.bash

High Level Launch Order:
roslaunch wheelchair_description wheelchair_bringup.launch -- Hokuyo, Joint States, Robot States, Laser Scan Filter, Odometry
roslaunch wheelchair_description wheelchair_merger.launch -- Laser Merger. Needs to be launched independently.
roslaunch wheelchair_navigation move_base_local.launch -- Navigation without a map.

Other Launch Files:
Hokuyo/Joint States/Robot States/Laser Scan Filter: roslaunch wheelchair_description wheelchair_description.launch

Laser Scan Merger: roslaunch wheelchair_description wheelchair_merger.launch

Laser Scan Matcher (Odometry): roslaunch wheelchair_description wheelchair_odom.launch

Dependances
---------------------
ros-indigo-robot-upstart
ros-indigo-scan-tools
ros-indigo-move-base

laser_scan_matcher
---------------------
$ cd [indigo catkin workspace directory]
$ git clone https://github.com/ccny-ros-pkg/scan_tools
$ sudo apt-get install libgsl0-dev
$ catkin_make
$ roslaunch laser_scan_matcher demo.launch



Simulation Bringup
---------------------
~/catkin_ws/src/wheelchair-automation/wheelchair_description/gazebo_spawn.bash
roslaunch wheelchair_description wheelchair_bringup.launch
roslaunch wheelchair_description wheelchair_merger.launch
roslaunch wheelchair_navigation move_base_local.launch

Bagfile Playback
---------------------
roslaunch wheelchair_description wheelchair_bringup.launch
roslaunch wheelchair_description wheelchair_merger.launch
rosbag play [bagfile]
