cmake_minimum_required(VERSION 2.8.3)
project(rqt_runstop)
find_package(catkin REQUIRED COMPONENTS
  rospy
  rqt_gui
  rqt_gui_py
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
