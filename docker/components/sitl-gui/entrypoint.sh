#!/bin/bash

source /opt/ros/kinetic/setup.bash
source ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default

export ROS_MASTER_URI=http://mavros-avoidance:11311

rviz -d /root/avoidance/resource/planning.rviz &
GAZEBO_MASTER_URI=http://sitl-avoidance-server:11345 gzclient --verbose
