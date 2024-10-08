#!/bin/bash

ros2 launch nav2_bringup bringup_launch.py \
    slam:="${SLAM:-True}" \
    params_file:=./navigation_params_"${ROS_DISTRO}".yaml \
    use_sim_time:=True
