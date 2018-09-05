wheelchair_description
======================

Package containing the base wheelchair information and control code

Launch files:
-------------

wheelchair_bringup.launch: Combined launch file for wheelchair_joy wheelchair_description and wheelchair_odom

wheelchair_description.launch: Launch file for TF data and state publishers

wheelchair_joy.launch: Launch file for physical controller nodes and scalers

wheelchair_merger.launch: Launch file for laserscan merger

wheelchair_nav_priorities.launch: Launch file for cmd_vel multiplexer and nodelet manager

wheelchair_odom.launch: Launch file for laser_scan_matcher

wheelchair_standard_core.launch: Launch file to start all basic wheelchair functions

wheelchair_vector_movement_core.launch: Launch file to start all basic wheelchair functions and vector field movement
