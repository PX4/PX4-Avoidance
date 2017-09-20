# Obstacle Detection and Avoidance
ROS node for sensor fusion and avoidance

# Prerequisites

## Default Installation for Ubuntu 14.04 and ROS Indigo

Follow installation guide from http://dev.px4.io/ to install ROS Indigo and Mavros (from source).

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
# Clone the repository
git clone https://github.com/PX4/avoidance.git
```

## Beta Installation for Ubuntu 16.04 and ROS Kinetic

Follow installation guide from http://dev.px4.io/ to install ROS Kinetic and Mavros (from source).

```bash
sudo apt-get update
sudo apt-get install libpcl1 ros-kinetic-octomap-*
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
git clone https://github.com/PX4/avoidance.git
```

# Running the Planner in Simulation

Follow installation guide from http://dev.px4.io/ to install Gazebo and SITL.

## Building the Code

```bash
# Source SITL and catkin
. ~/catkin_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/models
cd <Firmware_directory>
. $PWD/Tools/setup_gazebo.bash $PWD $PWD/<build_directory>
make posix_sitl_default gazebo
```

Now close Gazebo, the last line is just to generate necessary SDF-files.

```bash
catkin build
```

If ROS can't find PX4 or mavlink_sitl_gazebo, add the directories to ROS_PACKAGE_PATH, e.g.
```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:<Firmware_directory>
```

```bash
roslaunch avoidance global_planner_sitl_mavros.launch
```

You should now see the drone unarmed on the ground. To start flying, arm the drone and put it into offboard-mode.

```bash
# In another terminal
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
gz camera --camera-name=gzclient_camera --follow=iris # [Optional] to make Gazebo follow the drone
```

Now the ROS-node */path_handler_node* continuously publishes positions to the topic */mavros/setpoint_position/local*.
Initially the drone should just hover at 3.5m altitude.
To change the position without avoidance set the position with *2D Pose Estimate* in rviz.

To plan a new path set a new goal with *2D Nav Goal* in rviz. The the planned path should show up in rviz and the drone should follow the path.

If the drone does not follow the path properly, some tuning may be required in the file
*<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris*

If the planner is not working there are some parameters that can be tuned in *rqt reconfigure*


## Simulating stereo-vision
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


# Running the Planner on Hardware

The global planner uses the octomap_servers to get probabilistic information about the evironment.
The octomap_server needs a stream of point-clouds to generate the accumulated data.

## Generating Point-clouds from Depth-maps

In case the point-cloud stream already exists, this step can be skipped.

Assuming there already exists a stream of depth-maps on the ROS-topic <depthmap_topic>, we need to generate a corresponding stream of depth-maps.
Start by following the instructions from [PX4/disparity_to_point_cloud](https://github.com/PX4/disparity_to_point_cloud).
Now run the point-cloud generation with the parameters for the camera intrinsics:

```bash
rosrun disparity_to_point_cloud disparity_to_point_cloud_node \
    fx_:=fx fy_:=fy cx_:=cx cy_:=cy base_line_:=base_line disparity:=<depthmap_topic>
```

A stream of point-clouds should now be published to */point_cloud*.

## Running the planner

The planner can then be run can be run with

```bash
catkin build
roslaunch avoidance global_planner_offboard.launch point_cloud_topic:=<point_cloud_topic>
```  

# Running on Odroid

Read the [Running on Odroid](https://github.com/PX4/avoidance/blob/master/resource/odroid/) instructions
