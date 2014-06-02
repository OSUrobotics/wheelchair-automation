

rosrun xacro xacro.py urdf/wheelchair.xacro > urdf/model.urdf

trap 'kill $fooPid $barPid' EXIT

roslaunch gazebo_ros empty_world.launch & fooPid=$!

rosrun gazebo_ros spawn_model -file urdf/model.urdf -urdf -model wheelchair & barPid=$!

wait


