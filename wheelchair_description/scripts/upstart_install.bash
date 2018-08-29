
trap 'kill $corePid' EXIT

roscore & corePid=$!

rosrun robot_upstart install wheelchair_description/launch/wheelchair_full_startup.launch --job wheelchair

sleep 1

exit
