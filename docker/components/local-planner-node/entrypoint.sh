#!/bin/bash

source /root/catkin_ws/devel/setup.bash

export ROS_IP=`hostname -I`

# Wait until ROS master is started
until rostopic list; do sleep 1; done

LAUNCH_FILE=$1
shift

roslaunch /root/launch/${LAUNCH_FILE} $@ start_time:="$(date +'%Y%m%d_%I%M%S')"
