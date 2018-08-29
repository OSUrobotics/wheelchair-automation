#! /bin/bash

trap 'kill $mergerPid $bringPid $navPid' EXIT

roslaunch wheelchair_description wheelchair_bringup.launch &bringPid=$!

roslaunch wheelchair_description wheelchair_nav_priorities.launch &navPid=$!

sleep 3

roslaunch wheelchair_description wheelchair_merger.launch & mergerPid=$!

wait
