
#Not yet functional

rosrun xacro xacro.py urdf/wheelchair_gazebo.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid $extraPid $rvizPid $spawnPid $mergerPid $movePid' EXIT

roslaunch wheelchair_description wheelchair_world.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -model wheelchair -x -2 -y 1 & barPid=$!

sleep 10

roslaunch wheelchair_description wheelchair_bringup.launch & spawnPid=$!

sleep 10

roslaunch wheelchair_description wheelchair_merger.launch & mergerPid=$!

sleep 10

roslaunch wheelchair_navigation move_base_local.launch & movePid=$!

sleep 5

rosrun rviz rviz -d ./WheelchairSetup.rviz & rvizPid=$!
wait
