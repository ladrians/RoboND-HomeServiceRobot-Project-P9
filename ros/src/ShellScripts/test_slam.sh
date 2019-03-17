#! /bin/bash

echo 'Test SLAM...'

xterm -e "roslaunch ls_bot u.launch" &
sleep 10 &&
xterm -e "roslaunch ls_bot view_navigation.launch" &
sleep 3 &&
xterm -e "roslaunch ls_bot teleop.launch" &
sleep 3 &&
xterm -e "roslaunch ls_bot gmapping_demo.launch" &

