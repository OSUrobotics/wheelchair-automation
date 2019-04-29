wheelchair-automation
=====================

Code for automating a PERMOBIL wheelchair for the ALS foundation.

Package Descriptions
---------------------

+ <b><p>wheelchair_description:</p></b>
  --URDF and launch files for wheelchair initial bringup. Bash files for full simulation bringup.

+ <b><p>wheelchair_navigation:</p></b>
  --AMCL and Navigation parameters and launch files for the wheelchair.

+ <b><p>wheelchair_bringup:</p></b>
  --Full system bringup launch files

+ <b><p>wheelchair_mapping:</p></b>
  --Basic gmapping launch file and where maps should be saved.

+ <b><p>wheelchair_diagnostics:</p></b>
  --Files for wheelchair debug and diagnostics

+ <b><p>wheelchair_follower:</p></b>
  --Nothing ATM
+ <b><p>wheelchair_gui:</p></b>
  --Framework for a GUI

+ <b><p>wheelchair_power:</p></b>
  --Nothing ATM

+ <b><p>wheelchair_simulation:</p></b>
  --Launch files for spawning a wheelchair into Gazebo



High-Level System Bringup
--------------------
<h6>wheelchair-automation startup:</h6>

+ <b><p> wheelchair_standard_teleop_startup.launch:</p></b>
 --Starts up a basic teleop core

+ <b><p> wheelchair_standard_movebase_startup.launch:</p></b>
 --Starts up a teleop core with a navstack available

+ <b><p> wheelchair_standard_navigation_startup.launch:</p></b>
 --Starts up a teleop core with AMCL enabled navstack

+ <b><p> wheelchair_vector_teleop_startup.launch:</p></b>
 --Starts up a vector field core for teleop

+ <b><p> wheelchair_vector_movebase_startup.launch:</p></b>
 --Starts up a vector field core with navstack available

+ <b><p> wheelchair_vector_navigation_startup.launch:</p></b>
 --Starts up a vector field core with AMCL enabled navstack    

<b>Note:</b> Vector field is a prototype obstacle avoidance assistance program for teleoperation. It is by no means necessary and may result in odd behaviors.


<h6>Pim bridge bringup:</h6>

+ <b><p>rosrun ros_pim_bridge ros_pim_bridge_node</p></b>
 --Starts the PIM-ROS interface


System Access/System Setup
---------------------
<h6>For NUC-UP Board systems:</h6>
<p><b>System Access:</b></p>

<p><b>System Setup:</b></p>
<ol>
<li>On both NUC and UP board, install Ubuntu 14.04 (http://releases.ubuntu.com/14.04/)</li>
<li>On both, install ROS Indigo (http://wiki.ros.org/indigo/Installation/Ubuntu)</li>
<li>Set up catkin workspaces on both (http://wiki.ros.org/catkin/Tutorials/create_a_workspace)</li>
</ol>

On the NUC:
<ol>
<li>Git clone the Wheelchair Automation repository into your workspace’s src folder (https://github.com/OSUrobotics/wheelchair-automation.git)

<li>Install dependencies</li>
<ul>
<li>Option 1: Run the dependency install script found in the “wheelchair-automation” top-level directory</li>

<li>Option 2: Individually install all dependencies listed below</li>
</ul>
<li>Add the following lines to /etc/environment</li>
<ul>
<li>ROS_MASTER_URI=http://{NUC’s IP}:11311</li>
<li>ROS_IP={NUC’s IP}</li>
</ul>
</ol>

On the UP Board:
<ol>
<li>Clone the ros_pim_bridge package into your workspace’s src folder (https://zacharyianlee@bitbucket.org/zacharyianlee/ros_pim_bridge.git)</li>

<li>Add your user to the dialout group</li>
<ul>
<li>ros_pim_bridge needs access to /dev/ttyUSB0</li>
<li>If ttyUSB0 is still not accessible to the program, we’ve had success configuring udev to give the port proper permissions on startup</li>
</ul>
<li>Add the following lines to /etc/environment</li>
<ul>
<li>ROS_MASTER_URI=http://{NUC’s IP}:11311</li>
<li>ROS_IP={UP Board’s IP}</li>
</ul>
</ol>

<h6>For NUC-Only systems:</h6>
<p><b>System Access:</b></p>
<p><b>System Setup:</b></p>
<ol>
<li>Git clone the Wheelchair Automation repository into your workspace’s src folder (https://github.com/OSUrobotics/wheelchair-automation.git)

<li>Install dependencies</li>
<ul>
<li>Option 1: Run the dependency install script found in the “wheelchair-automation” top-level directory</li>

<li>Option 2: Individually install all dependencies listed below</li>
</ul>
<li>Add the following lines to /etc/environment</li>
<ul>
<li>ROS_MASTER_URI=http://{NUC’s IP}:11311</li>
<li>ROS_IP={NUC’s IP}</li>
</ul>
<li>Clone the ros_pim_bridge package into your workspace’s src folder (https://zacharyianlee@bitbucket.org/zacharyianlee/ros_pim_bridge.git)</li>

<li>Add your user to the dialout group</li>
<ul>
<li>ros_pim_bridge needs access to /dev/ttyUSB0</li>
<li>If ttyUSB0 is still not accessible to the program, we’ve had success configuring udev to give the port proper permissions on startup</li>
</ul>
</ol>

Dependencies
---------------------
+ sudo apt-get install ros-indigo-robot-upstart  
+ sudo apt-get install ros-indigo-scan-tools   
+ sudo apt-get install ros-indigo-move-base  
+ sudo apt-get install ros-indigo-urg-node
+ sudo apt-get install ros-indigo-teleop-twist-joy
+ sudo apt-get install ros-indigo-teleop-twist-keyboard
+ sudo apt-get install ros-indigo-spacenav-node
+ sudo apt-get install ros-indigo-yocs-cmd-vel-mux
+ sudo apt-get install ros-indigo-joy
+ sudo apt-get install ros-indigo-laser-geometry
+ sudo apt-get install ros-indigo-laser-scan-matcher
+ ira_laser_tools

<b>ira_laser_tools installation:</b>
<ol>
<li>cd [indigo catkin workspace directory]  </li>
<li>git clone https://github.com/iralabdisco/ira_laser_tools.git</li>
<li>catkin_make  </li>
</ol>
