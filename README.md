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

Controller:

ssh into iaso (raspberry pi).

sudo su

rosrun controller pro_controller2c.py
