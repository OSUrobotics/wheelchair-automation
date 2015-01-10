wheelchair-automation
=====================

Code for automating a PERMOBIL wheelchair for the ALS foundation.

Package Descriptions
---------------------

wheelchair_description: URDF and launch files for wheelchair initial bringup.

wheelchair_automation: AMCL and Navigation for the wheelchair.

wheelchair_bringup: TODO

wheelchair_mapping: Basic gmapping launch file and where maps should be saved. (TODO: Get rid of? Probably.)

Useful launch files
---------------------

Gazebo (Only for simulation): bash gazebo_spawn_swivel_willow.bash 

Hokuyo/Joint States/Robot States/Laser Scan Merger: roslaunch wheelchair_description wheelchair_description.launch

Laser Scan Matcher (Odometry): roslaunch wheelchair_description wheelchair_odom.launch

Note that Odometry becomes inaccurate when spinning in place. Causes localization to become inaccurate after a few full spins.

AMCL: roslaunch wheelchair_automation amcl.launch

Navigation: roslaunch wheelchair_automation navigation.launch

