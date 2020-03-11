#!/usr/bin/bash

sudo chmod 0666 /dev/ttyUSB0
gnome-terminal -- bash -c "rosrun dobot_driver DobotServer /dev/ttyUSB0; bash" 
sleep 3
gnome-terminal -- bash -c "roslaunch dobot_coordinate_transformer test.launch; bash" 

