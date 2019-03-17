#! /bin/bash

echo ' '

xterm -x "roslaunch ls_bot u.launch" &
sleep 3 &&
xterm -x "roslaunch ls_bot teleop.launch" &
sleep 3 &&

echo ' '
read -p 'Press any key to continue to mapping... ' -n1 -s

xterm -x "roslaunch ls_bot mapping.launch simulation:=true" &

