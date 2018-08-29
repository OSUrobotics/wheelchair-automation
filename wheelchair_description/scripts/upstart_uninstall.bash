
trap 'kill $corePid' EXIT

roscore & corePid=$!

rosrun robot_upstart uninstall wheelchair

sleep 1

exit
