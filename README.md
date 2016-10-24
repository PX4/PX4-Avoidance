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
sudo apt-get install ros-kinetic-octomap-mapping

# If octomap-mapping is not found, then it needs to be compiled from source
cd <catkin_directory>/src
git clone https://github.com/OctoMap/octomap_mapping.git
```

# Building the Code

Now clone the repository into the catkin workspace and build
```bash
# Source SITL and catkin
cd <catkin_directory>
source devel/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:<catkin_directory>/src/detection/models
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
To simulate obstacle avoidance with stereo-cameras, make sure `stereo-image-proc` is installed.
```bash
	# Install stereo-image-proc
	sudo apt-get install ros-$ROS_DISTRO-stereo-image-proc
	# Launch simulation
	roslaunch avoidance global_planner_stereo.launch
```
Simulated stereo-vision is prone to errors due to artificial texture, which may make obstacles appear extremly close to the camera. For best results choose a more natural [world](https://github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin/tree/master/ardupilot_sitl_gazebo_plugin/worlds/outdoor_village).
```bash
	git clone https://github.com/AurelienRoy/^Cdupilot_sitl_gazebo_plugin.git
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