wheelchair-automation
=====================

Code for automating a PERMOBIL wheelchair for the ALS foundation.

Package Descriptions
---------------------

wheelchair_description: URDF and launch files for wheelchair initial bringup. Bash files for full simulation bringup.

wheelchair_navigation: AMCL and Navigation parameters for the wheelchair.

wheelchair_bringup: Demo launch files.

wheelchair_mapping: Basic gmapping launch file and where maps should be saved.

wheelchair_diagnostics: Files for wheelchair debug and diagnostics

wheelchair_follower:

wheelchair_gui: Framework for a GUI

wheelchair_mapping: Launch file for gmapping with wheelchair

wheelchair_power: Nothing ATM

wheelchair_simulation: Launch files for spawning a wheelchair into Gazebo

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

+ <p>roslaunch wheelchair_description</p> wheelchair_merger.launch</p>
 -- Laser Scan Merger

+ <p>roslaunch wheelchair_description wheelchair_odom.launch</p>
 -- Laser Scan Matcher (Odometry)

+ <p>roslaunch wheelchair_description wheelchair_full_startup.launch</p>
 -- Full teleop startup

Dependencies
---------------------
+ sudo apt-get install ros-indigo-robot-upstart  
+ sudo apt-get install ros-indigo-scan-tools   
+ sudo apt-get install ros-indigo-move-base  
+ sudo apt-get install ros-indigo-urg-node
+ sudo apt-get install ros-indigo-teleop-twist-joy
+ sudo apt-get install ros-indigo-spacenav-node
+ sudo apt-get install ros-indigo-yocs-cmd-vel-mux
+ sudo apt-get install ros-indigo-joy
+ sudo apt-get install ros-indigo-laser-geometry
+ ira_laser_tools

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

Simple Teleop Bringup
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

Full Teleop Bringup
-------------------
1. roslaunch wheelchair_description wheelchair_full_startup.launch
