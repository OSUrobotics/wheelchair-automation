

rosrun xacro xacro.py urdf/wheelchair_gazebo.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid $extraPid $rvizPid $spawnPid $mergerPid $movePid' EXIT

roslaunch wheelchair_description wheelchair_world.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -model wheelchair -x -0 -y 0 & barPid=$!

sleep 8

roslaunch wheelchair_description wheelchair_bringup.launch & spawnPid=$!

sleep 8

rosrun rviz rviz -d ./WheelchairSetup.rviz & rvizPid=$!
wait
