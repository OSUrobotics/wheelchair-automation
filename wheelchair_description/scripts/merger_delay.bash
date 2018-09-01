#! /bin/bash

trap 'kill $mergerPid' EXIT

sleep 3

roslaunch wheelchair_description wheelchair_merger.launch & mergerPid=$!

wait
