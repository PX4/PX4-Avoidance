#!/bin/bash

source /opt/ros/kinetic/setup.bash

export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${FIRMWARE_DIR}

source ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default

export ROS_MASTER_URI=http://mavros-avoidance:11311
export ROS_IP=`hostname -I`

# Wait until ROS master is started
until rostopic list; do sleep 1; done

roslaunch ${WORKSPACE_DIR}/px4_gazebo.launch gui:="false" $1
