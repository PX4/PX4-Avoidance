#!/bin/bash

cd ~/src/Firmware && git submodule update --init --recursive && DONT_RUN=1 make px4_sitl_default gazebo

cd ~/catkin_ws/src && git clone https://github.com/PX4/avoidance.git && catkin build

echo "source ~/src/Firmware/Tools/setup_gazebo.bash ~/src/Firmware ~/src/Firmware/build/px4_sitl_default > /dev/null
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/src/Firmware:~/src/Firmware/Tools/sitl_gazebo"  >> ~/.bashrc
