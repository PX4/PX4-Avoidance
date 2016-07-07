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
rosdep install octomap_mapping
rosmake octomap_mapping
```

## Beta Installation for Ubuntu 16.04 and ROS Kinetic

Follow installation guide from http://dev.px4.io/ to install ROS Kinetic, Gazebo, SITL and Mavros (from source).

```bash
# Install PCL
sudo apt-get update
sudo apt-get install libpcl1
```

```bash
# Install the Octomap Library
sudo apt-get install ros-kinetic-octomap-*
rosdep install octomap
rosmake octomap
```

# Building the Code

Now clone the repository into the catkin workspace and build
```bash
# Source SITL and catkin
cd <Firmware_dir>
source integrationtests/setup_gazebo_ros.bash $(pwd)
cd <catkin_directory>
source devel/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<catkin_directory>/src/detection/models
```

```bash
roslaunch avoidance global_planner_sitl_mavros.launch
```

If the drone does not follow the path properly, some tuning may be required in the file 
<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris 
