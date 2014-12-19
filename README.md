wheelchair-automation
=====================

Code for automating a PERMOBIL wheelchair for the ALS foundation.

Needed packages:
libgsl0-dev
ros-hydro-transmission-interface
ros-hydro-joint-limits-interface
ros-hydro-joint-trajectory-controller
ros-hydro-controller-manager

wheelchair controlling:

Gazebo: bash gazebo_spawn_swivel_willow.bash 
Joints: roslaunch wheelchair_description wheelchair_description.launch
Fake Odom: roslaunch wheelchair_description wheelchair_odom.launch
AMCL: roslaunch test_simulation amcl.launch
Navigation: roslaunch test_simulation navigation.launch
