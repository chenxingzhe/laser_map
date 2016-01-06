#!/bin/bash
cd ~/laser_map
source ./devel/setup.bash
v4l2-ctl --set-ctrl=exposure_auto=1
v4l2-ctl --set-ctrl=exposure_absolute=50
roslaunch beginner_tutorials map.launch
