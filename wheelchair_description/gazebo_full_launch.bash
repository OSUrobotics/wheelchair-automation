
#Not yet functional

rosrun xacro xacro.py urdf/wheelchair_gazebo.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid $extraPid $rvizPid' EXIT

roslaunch wheelchair_description wheelchair_world.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -model wheelchair -x -2 -y 1 & barPid=$!

roslaunch wheelchair_bringup wheelchair_full_simulation.launch & extraPid=$!

rosrun rviz rviz & rvizPid=$!
wait
