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

+ <p>roslaunch wheelchair_description wheelchair_bringup.launch</p>
-- Hokuyo, Joint States, Robot States, Laser Scan Filter, Odometry

+ <p>roslaunch wheelchair_description wheelchair_merger.launch</p>
 -- Laser Merger. Needs to be launched independently.

+ <p>roslaunch wheelchair_navigation move_base_local.launch</p>
 -- Navigation without a map.

Other Launch Files:
+ <p>roslaunch wheelchair_description wheelchair_description.launch</p>
 -- Hokuyo/Joint States/Robot States/Laser Scan Filter.

+ <p>roslaunch wheelchair_description</p> wheelchair_merger.launch
 -- Laser Scan Merger

+ <p>roslaunch wheelchair_description wheelchair_odom.launch</p>
 -- Laser Scan Matcher (Odometry)

Dependances
---------------------
+ ros-indigo-robot-upstart  
+ ros-indigo-scan-tools   
+ ros-indigo-move-base  

laser_scan_matcher
---------------------
1. cd [indigo catkin workspace directory]  
2. git clone https://github.com/ccny-ros-pkg/scan_tools  
3. sudo apt-get install libgsl0-dev  
4. catkin_make  
5. roslaunch laser_scan_matcher demo.launch  

Simulation Bringup
---------------------
1. ~/catkin_ws/src/wheelchair-automation/wheelchair_description/gazebo_spawn.bash  
2. roslaunch wheelchair_description wheelchair_bringup.launch  
3. roslaunch wheelchair_description wheelchair_merger.launch  
4. roslaunch wheelchair_navigation move_base_local.launch  

Bagfile Playback
---------------------
1. roslaunch wheelchair_description wheelchair_bringup.launch
2. roslaunch wheelchair_description wheelchair_merger.launch
3. rosbag play [bagfile]

Simple Telep Bringup
---------------------
1. roslaunch wheelchair_description wheelchair_bringup.launch
2. roslaunch wheelchair_description wheelchair_merger.launch

Simple Local Nav Bringup
---------------------
1. roslaunch wheelchair_description wheelchair_bringup.launch
2. roslaunch wheelchair_description wheelchair_merger.launch
3. roslaunch wheelchair_navigation move_base_local.launch

Simple Follow Bringup
---------------------
1. roslaunch wheelchair_description wheelchair_bringup.launch
2. roslaunch wheelchair_description wheelchair_merger.launch
2. roslaunch wheelchair_follower follow_filter.launch
3. roslaunch wheelchair_navigation move_base_local_follower.launch
