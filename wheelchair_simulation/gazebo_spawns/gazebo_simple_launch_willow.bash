
#Not yet functional

rosrun xacro xacro.py urdf/wheelchair_gazebo.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid $extraPid $rvizPid $spawnPid' EXIT

roslaunch wheelchair_description wheelchair_world_willow.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -model wheelchair -x -2 -y 1 & barPid=$!

sleep 15

roslaunch wheelchair_description wheelchair_bringup.launch & spawnPid=$!

sleep 10

rosrun rviz rviz -d ./WheelchairSetup.rviz & rvizPid=$!
wait
