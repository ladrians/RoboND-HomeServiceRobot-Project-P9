#! /bin/bash

echo 'Test Navigation...'

xterm -e "roslaunch ls_bot u.launch" &
sleep 10 &&
xterm -e "roslaunch ls_bot view_navigation.launch" &
sleep 3 &&
xterm -e "roslaunch ls_bot amcl.launch" &

