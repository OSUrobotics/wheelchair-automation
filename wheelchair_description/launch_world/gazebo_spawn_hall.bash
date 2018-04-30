

rosrun xacro xacro.py urdf/wheelchair_gazebo.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid' EXIT

roslaunch wheelchair_description wheelchair_world_hall.launch & fooPid=$!


#rosrun gazebo_ros spawn_model -file ../urdf/model.urdf -urdf -model wheelchair -x -3 -y 0 & barPid=$!
rosrun gazebo_ros spawn_model -file ../urdf/model.urdf -urdf -model wheelchair -x -5 -y 0 & barPid=$!

wait


