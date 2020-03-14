#!/usr/bin/bash

sudo chmod 0666 /dev/ttyUSB0
gnome-terminal -- bash -c "roslaunch astra_camera stereo_s.launch; bash"
sleep 1
gnome-terminal -- bash -c "rosrun dobot_coordinate_transformer red_point_detection; bash"
