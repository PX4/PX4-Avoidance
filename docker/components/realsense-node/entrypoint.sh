#!/bin/bash

source /opt/ros/kinetic/setup.bash

# Wait until ROS master is started
until rostopic list; do sleep 1; done

roslaunch realsense_camera r200_nodelet_rgbd.launch
