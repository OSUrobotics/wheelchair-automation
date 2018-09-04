#! /bin/bash

trap 'rosrun map_server map_saver -f lab.pgm &savePid=$!' EXIT

trap 'kill $mapPid' EXIT

roslaunch wheelchair_mapping wheelchair_mapping_gmapping.launch &mapPid=$!

wait
