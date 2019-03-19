#! /bin/bash

echo ' '

xterm -x "roslaunch ls_bot u.launch" &
sleep 3
xterm -x "roslaunch ls_bot teleop.launch" &
sleep 3
xterm -x "roslaunch ls_bot mapping.launch simulation:=true" &

