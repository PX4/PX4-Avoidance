#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_IP=${ROS_IP:-`hostname -I`}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://`hostname -I`:11311}

roscore -p 11311 &
sleep 1

roslaunch mavros -p 11311 px4.launch $1
