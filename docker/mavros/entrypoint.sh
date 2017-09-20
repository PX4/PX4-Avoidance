#!/bin/bash

source ~/catkin_ws/devel/setup.bash
export ROS_IP=`hostname -I`
export ROS_MASTER_URI=http://`hostname -I`:11311

roscore -p 11311 &
sleep 1
rosparam set use_sim_time true

roslaunch mavros -p 11311 px4.launch $1
