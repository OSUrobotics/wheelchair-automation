sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install openssh-server vim git htop

sudo apt-get install ros-indigo-desktop-full ros-indigo-robot-upstart ros-indigo-move-base ros-indigo-urg-node ros-indigo-teleop-twist-joy ros-indigo-teleop-twist-keyboard ros-indigo-spacenav-node ros-indigo-yocs-cmd-vel-mux ros-indigo-joy ros-indigo-laser-geometry ros-indigo-amcl ros-indigo-serial
launch-cache search ros-indigo

sudo rosdep init
rosdep update

source /opt/ros/indigo/setup.bash

sudo apt-get install python-rosinstall

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws

catkin_make

source ./devel/setup.bash

cd src

git clone -b M3LaunchReordering https://github.com/OSUrobotics/wheelchair-automation.git
# git clone -b indigo https://github.com/ccny-ros-pkg/scan_tools.git
git clone -b master https://github.com/iralabdisco/ira_laser_tools.git
git clone -b master https://github.com/benjaminnarin/laser_scan_matcher_odom
git clone -b master https://bitbucket.org/zacharyianlee/ros_pim_bridge/src/master/
cd ../

catkin_make
