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
cd $HOME/catkin_ws
source devel/setup.bash
source integrationtests/setup_gazebo_ros.bash $HOME/catkin_ws/src/Firmware
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/detection/models
```

Make sure that the relevant Firmware directories are known by ROS and Gazebo
```bash
	# May or may not be needed, depends on the setup
	source <Firmware_directory>/Tools/setup_gazebo.bash <Firmware_directory> <Firmware_directory>/build_posix_sitl_default
	# If ROS can't find PX4 or mavlink_sitl_gazebo, add the directories to ROS_PACKAGE_PATH, e.g.
	export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<Firmware_directory>
``` 

```bash
roslaunch avoidance global_planner_sitl_mavros.launch
```

If the drone does not follow the path properly, some tuning may be required in the file 
<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris 


# Running on Odroid
Connect to the access point, name:px4_outdoor

Log in to the Odroid
```bash
ssh odroid@192.168.2.239
	password: odroid
```

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttySAC0:921600
roslaunch uvc_ros_driver uvc_ros_driver.launch calibrationMode:=1
roslaunch disparity_to_point_cloud d2pcloud.launch
roslaunch avoidance global_planner_offboard.launch
```