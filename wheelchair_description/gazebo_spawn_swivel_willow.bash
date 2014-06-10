

rosrun xacro xacro.py urdf/wheelchair_with_swivels.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid' EXIT

roslaunch gazebo_ros willowgarage_world.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -z 0.5 -model wheelchair & barPid=$!

sleep 10

roslaunch wheelchair_description wheelchair_description.launch

wait

