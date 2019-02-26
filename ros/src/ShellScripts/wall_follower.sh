#! /bin/bash

echo ' '

x-terminal-emulator -x roslaunch wall_follower u.launch 2>/dev/null &

sleep 3 &&

x-terminal-emulator -x roslaunch wall_follower teleop.launch 2>/dev/null &

sleep 3 &&

echo ' '
read -p 'Press any key to continue to mapping... ' -n1 -s

x-terminal-emulator -x roslaunch wall_follower mapping.launch simulation:=true 2>/dev/null &
# sleep 3 &&
#x-terminal-emulator -e roslaunch wall_follower rviz.launch 2>/dev/null

echo ' '
echo 'Script Completed'
echo ' '
