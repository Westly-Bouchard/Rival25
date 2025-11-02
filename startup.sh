#!/bin/bash

# device to look for
MAC="10:C7:35:AB:29:CB"
ROS_SOURCE="source ~/Rival25/install/setup.bash"
ROS_LAUNCH="ros2 launch swerve swerve_module.launch.py"

while true; do
    if bluetoothctl info "$MAC" | grep -q "Connected: yes"; then
        exec $ROS_SOURCE
        exec $ROS_LAUNCH
    fi

    sleep 1
done