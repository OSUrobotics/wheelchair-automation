#! /bin/bash

trap 'kill $joyPid $mergerPid $odomPid $navPid' EXIT

roslaunch wheelchair_description wheelchair_odom.launch &odomPid=$!

sleep 5

roslaunch wheelchair_description wheelchair_joy.launch & joyPid=$!

sleep 5

roslaunch wheelchair_description wheelchair_nav_priorities.launch &navPid=$!

sleep 5

roslaunch wheelchair_description wheelchair_merger.launch & mergerPid=$!

wait
