#! /bin/bash

xterm -e "roslaunch ls_bot u.launch" &
sleep 10
xterm -e "roslaunch ls_bot view_navigation.launch" &
sleep 3
xterm -e "roslaunch ls_bot amcl.launch" &
sleep 5
xterm -e "rosrun pick_objects pick_objects.py" &

