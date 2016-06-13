# avoidance
ROS node for sensor fusion and avoidance

## Install Prerequisites

Follow installation guide from http://dev.px4.io/ to install ROS Indigo, Gazebo 6, SITL and Mavros.

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

```bash
# Source SITL and catkin
cd <Firmware_dir>
source integrationtests/setup_gazebo_ros.bash $(pwd)
cd <catkin_directory>
source devel/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<catkin_directory>/src/detection/models
```

```bash
roslaunch avoidance global_planner

```

If the drone does not follow the path properly, some tuning may be required in the file 
<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris 