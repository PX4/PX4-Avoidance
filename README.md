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


# Simulating stereo-vision
To simulate obstacle avoidance with stereo-cameras `stereo-image-proc` is must be installed.
```bash
# Launch simulation
roslaunch avoidance global_planner_stereo.launch
```
Simulated stereo-vision is prone to errors due to artificial texture, which may make obstacles appear extremly close to the camera. For best results choose a more natural [world](https://github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin/tree/master/ardupilot_sitl_gazebo_plugin/worlds/outdoor_village).
```bash
git clone https://github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin.git
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin/meshes/meshes_outdoor
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH:$(pwd)/ardupilot_sitl_gazebo_plugin/ardupilot_sitl_gazebo_plugin" 
```
 
The disparity map from `stereo-image-proc` is published as a
[stereo_msgs/DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html) message, which is not supported by rviz or rqt. To visualize the message, either run
```bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
```
or publish the DisparityImage as a simple sensor_msgs/Image 
```bash
rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image 'm.image'
```
Now the disparity map can be visualized by rviz or rqt under the topic /stereo/disparity_image.


# Running on Odroid

Read the [Running on Odroid](https://github.com/PX4/avoidance/blob/master/resource/odroid/) instructions