#!/bin/bash

source /opt/ros/jazzy/setup.bash

cd /home/west/Rival25
git clone
colcon build --packages-select swerve

source install/setup.bash

# device to look for
MAC="10:C7:35:AB:29:CB"
ROS_LAUNCH="ros2 launch swerve swerve_module.launch.py"

while true; do
    if bluetoothctl info "$MAC" | grep -q "Connected: yes"; then
        exec $ROS_LAUNCH
    fi

    sleep 1
done