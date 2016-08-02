# Obstacle Detection and Avoidance
ROS node for sensor fusion and avoidance

# Prerequisites

## Default Installation for Ubuntu 14.04 and ROS Indigo

Follow installation guide from http://dev.px4.io/ to install ROS Indigo, Gazebo, SITL and Mavros (from source).

```bash
# Install PCL
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

```bash
# Install the Octomap Library
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
rosdep install octomap_mapping
rosmake octomap_mapping
```

## Beta Installation for Ubuntu 16.04 and ROS Kinetic

Follow installation guide from http://dev.px4.io/ to install ROS Kinetic, Gazebo, SITL and Mavros (from source).

```bash
sudo apt-get update
sudo apt-get install libpcl1 ros-kinetic-octomap-*
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
catkin_build
```

# Building the Code

Now clone the repository into the catkin workspace and build
```bash
# Source SITL and catkin
cd $HOME/catkin_ws/src/Firmware
source integrationtests/setup_gazebo_ros.bash $(pwd)
cd $HOME/catkin_ws
source devel/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/detection/models
```

```bash
roslaunch avoidance global_planner_sitl_mavros.launch
```

If the drone does not follow the path properly, some tuning may be required in the file 
<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris 
