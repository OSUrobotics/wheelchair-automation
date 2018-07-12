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

Dependencies
---------------------
+ ros-indigo-robot-upstart  
+ ros-indigo-scan-tools   
+ ros-indigo-move-base  
+ ros-urg-node
+ ros-teleop-twist-joy

laser_scan_matcher
---------------------
1. cd [indigo catkin workspace directory]  
2. git clone https://github.com/ccny-ros-pkg/scan_tools  
3. sudo apt-get install libgsl0-dev  
4. catkin_make  
5. roslaunch laser_scan_matcher demo.launch  

ira_laser_tools
---------------------
1. cd [indigo catkin workspace directory]  
2. git clone https://github.com/iralabdisco/ira_laser_tools.git
3. catkin_make  

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

System Access/System Setup
---------------------
1. Connect to the Wifi:
  + chironLAN2 (2.4 Ghz)
  + chironLAN5 (5 Ghz)
2. If internet is needed, plug ethernet cable to port 1 on the hAP ac
3. SSH into Intel NUC and Raspberry Pi:
  + ssh artemis@192.168.1.200
  + ssh iaso@192.168.1.105
4. Run one of the following bringup launch file groups (teleop, nav, follow) on the NUC
5. Run interface launch on Raspberry Pi (must be run as root, roscore is on NUC)
  + sudo su root
  + rosrun controller pro_controller_i2c.py

Simple Telep Bringup
---------------------
1. Connect to the Wifi:
  + chironLAN2 (2.4 Ghz)
  + chironLAN5 (5 Ghz)
2. If internet is needed, plug ethernet cable to port 1 on the hAP ac
3. SSH into Intel NUC and Raspberry Pi:
  + ssh artemis@192.168.1.200
  + ssh iaso@192.168.1.105
4. Run one of the following bringup launch file groups (teleop, nav, follow) on the NUC
5. Run interface launch on Raspberry Pi (must be run as root, roscore is on NUC)
  + sudo su root
  + rosrun controller pro_controller_i2c.py

Simple Teleop Bringup
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
