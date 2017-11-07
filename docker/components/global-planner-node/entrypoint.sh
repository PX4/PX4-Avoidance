#!/bin/bash

source /root/catkin_ws/devel/setup.bash

export ROS_IP=`hostname -I`

# Wait until ROS master is started
until rostopic list; do sleep 1; done

roslaunch /root/launch/${1} $2
