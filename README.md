# Obstacle Detection and Avoidance
PX4 computer vision algorithms packaged as ROS nodes for depth sensor fusion and obstacle avoidance. This repository contains two different implementations, targeting different use cases:

  * A local, VFH+ based planner that plans (including some history) in a vector field histogram
  * A global, graph based planner that plans in a traditional occupancy grid
  
**The full PX4 Developer documentation is available on: https://dev.px4.io and includes more computer vision (including VIO) examples.**

## Bi-weekly Dev Call
The active developers of the avoidance repo are syncing up on a bi-weekly basis on the WG dev call.

* Join the call: https://zoom.us/j/506512712
* Time: Tuesday, 10AM CET
* Meeting ID: 506512712
* Dronecode calendar: https://www.dronecode.org/calendar/

[![PX4 Avoidance video](http://img.youtube.com/vi/VqZkAWSl_U0/0.jpg)](https://www.youtube.com/watch?v=VqZkAWSl_U0)

# Quick Start with Docker

A ROS container based on Ubuntu 16.04 has been created and can be used to quickly try the simulation, as a demo. Running it is as simple as installing docker and docker-compose, and running `$ docker-compose up` from the right folder. Find the corresponding instructions [here](docker/demo).

For __deployment__ instructions, check "[Deploying with Docker](docker#deploying-with-docker)".

If you want to leverage docker in your __development__ environment, check the "[Developing with Docker](docker#developing-with-docker)" section.

# Prerequisites

## Installation for Ubuntu 16.04 and ROS Kinetic

This is a step-by-step guide to install and build all the prerequisites for running this module on Ubuntu 16.04. You might want to skip some of them if your system is already partially installed. A corresponding docker container is defined [here](docker/ubuntu/Dockerfile) as reference.

Note that in the following instructions, we assume your catkin workspace (in which we will build the avoidance module) is in `~/catkin_ws`, and the PX4 Firmware directory is `~/Firmware`. Feel free to adapt this to your situation.

1. Add ROS to sources.list.

```bash
echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
apt-get update
```

2. Install gazebo with ROS (many dependencies including gazebo7 will come with those two packages).

```bash
apt install ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-ros-pkgs

# Source ROS
source /opt/ros/kinetic/setup.bash
```

3. Initialize rosdep.

```bash
rosdep init
rosdep update
```

4. Install catkin and create your catkin workspace directory.

```bash
apt install python-catkin-tools
mkdir -p ~/catkin_ws/src
```

5. Install mavros. The package coming from the ROS repository should be fine. Just in case, instructions to install it from sources can be found here: https://dev.px4.io/en/ros/mavros_installation.html.

```bash
apt install ros-kinetic-mavros
```

6. Install avoidance module dependencies (pointcloud library and octomap).

```bash
apt install libpcl1 ros-kinetic-octomap-*
```

7. Clone this repository in your catkin workspace in order to build the avoidance node.

```bash
cd ~/catkin_ws/src
git clone https://github.com/PX4/avoidance.git
```

8. Actually build the avoidance node.

```bash
catkin build -w ~/catkin_ws
```

If it fails, try to build again. It seems like it sometimes fails in an undeterministic way.

Note that you can build the node in release mode this way:

```bash
catkin build -w ~/catkin_ws --cmake-args -DCMAKE_BUILD_TYPE=Release
```

9. Clone the PX4 Firmware and all its submodules (it may take some time).

```bash
cd ~
git clone https://github.com/PX4/Firmware.git
cd ~/Firmware
git submodule update --init --recursive
```

10. Install Firmware dependencies.

```bash
apt install libopencv-dev python-jinja2 protobuf-compiler
```

11. We will now build the Firmware once in order to generate SDF model files for Gazebo. This step will actually run a simulation that you can directly quit.

```bash
# Add the models from the avoidance module to GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/sim/models

# This is necessary to prevent some Qt-related errors (feel free to try to omit it)
export QT_X11_NO_MITSHM=1

# Setup some more Gazebo-related environment variables
. ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/posix_sitl_default

# Build and run simulation
make posix_sitl_default gazebo
```

12. Add the Firmware directory to ROS_PACKAGE_PATH so that ROS can start the PX4.

```bash
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
```

You should now be ready to run the simulation.

## Installation for Ubuntu 14.04 and ROS Indigo

A full step-by-step guide is not available for this configuration. You might find the [instructions for Ubuntu 16.04 and ROS Kinetic](#installation-for-ubuntu-1604-and-ros-kinetic) useful, though. Another source of information is also the [PX4 dev guide](http://dev.px4.io).

Notable differences with ROS Kinetic are:

* Installing the pointcloud library requires another package from another repository:

```bash
# Install PCL
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```

* Octomap has different packages for Indigo:

```bash
# Install the Octomap Library
sudo apt-get install ros-indigo-octomap ros-indigo-octomap-mapping
```

# Running the Planner in Simulation

## Simulating a depth camera

1. Make sure that you have sources the catkin setup.bash from your catkin workspace.

```bash
source ~/catkin_ws/devel/setup.bash
```

2. Run the simulation.

```bash
roslaunch global_planner global_planner_sitl_mavros.launch
```

You should now see the drone unarmed on the ground, and the octomap should show 2 red arrows and the visible world, as pictured below.

![Screenshot showing gazebo and rviz](docs/simulation_screenshot.png)

To start flying, put the drone in OFFBOARD mode and arm it. The avoidance node will then take control of it.

```bash
# In another terminal
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

Initially the drone should just hover at 3.5m altitude.

From the command line, you can also make Gazebo follow the drone, if you want.

```bash
gz camera --camera-name=gzclient_camera --follow=iris
```

During the simulation, the ROS node *"/path_handler_node"* continuously publishes positions to the topic *"/mavros/setpoint_position/local"*.

The graph of the ROS nodes is shown below:

![Graph showing the ROS nodes and their links](docs/rqt_graph.png)

One can plan a new path by setting a new goal with the *2D Nav Goal* button in rviz. The planned path should show up in rviz and the drone should follow the path, updating it when obstacles are detected. It is also possible to set a goal without using the obstacle avoidance (i.e. the drone will go straight to this goal and potentially collide with obstacles). To do so, set the position with the *2D Pose Estimate* button in rviz.


## Simulating stereo-vision
To simulate obstacle avoidance with stereo-cameras, the package `stereo-image-proc` must be installed.

```bash
# Launch simulation
roslaunch global_planner global_planner_stereo.launch
```

Simulated stereo-vision is prone to errors due to artificial texture, which may make obstacles appear extremely close to the camera. For better results, choose a more natural [world](https://github.com/PX4/avoidance/blob/master/sim/worlds/simple_obstacle.world).

The disparity map from `stereo-image-proc` is published as a [stereo_msgs/DisparityImage](http://docs.ros.org/api/stereo_msgs/html/msg/DisparityImage.html) message, which is not supported by rviz or rqt. To visualize the message, either run:

```bash
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
```

or publish the DisparityImage as a simple sensor_msgs/Image

```bash
rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image 'm.image'
```

Now the disparity map can be visualized by rviz or rqt under the topic */stereo/disparity_image*.

## Running the Local Planner

The local planner is based on the [3DVFH+](http://ceur-ws.org/Vol-1319/morse14_paper_08.pdf) algorithm. To run the algorithm simulating stereo vision

```bash
# if stereo-image-proc not yet installed
sudo apt-get install ros-kinetic-stereo-image-proc

roslaunch local_planner local_planner_stereo.launch
```
Another option is to simulate a kinect depth sensor:

```bash
roslaunch local_planner local_planner_depth-camera.lauch
```

You will see the Iris drone unarmed in the Gazebo world. To start flying, put the drone in OFFBOARD mode and arm it. The avoidance node will then take control of it.

```bash
# In another terminal
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
```

The drone will first change its altitude to reach the goal height. It is possible to modify the goal altitude with `rqt_reconfigure` GUI.
![Screenshot rqt_reconfigure goal height](docs/lp_goal_height.png)
Then the drone will start moving towards the goal. The default x, y goal position can be changed in Rviz by clicking on the 2D Nav Goal button and then chosing the new goal x and y position by clicking on the visualized gray space. If the goal has been set correctly, a yellow sphere will appear where you have clicked in the grey world.
![Screenshot rviz goal selection](docs/lp_goal_rviz.png)

### How to recover the logs

It is possible to log the point cloud, the polar histogram, the waypoints and the drone position when simulating the local planner. In order to (dis)enable logging, you need to set the following arguments to `false`/`true` in the `local_planner_depth-camera.launch` launch file:
```bash
<arg name="record_point_cloud" default="true" />
<arg name="record_histogram"   default="true" />
<arg name="record_waypoints"   default="true" />
<arg name="record_position"    default="true" />
```
You can retrieve the logs in the `~/.ros/` folder of you computer.

## Troubleshooting

### I see the drone position in rviz (shown as a red arrow), but the world around is empty
Check that some camera topics (including */camera/depth/points*) are published with the following command:

```bash
rostopic list | grep camera
```

If */camera/depth/points* is the only one listed, it may be a sign that gazebo is not actually publishing data from the simulated depth camera. Verify this claim by running:

```bash
rostopic echo /camera/depth/points
```

When everything runs correctly, the previous command should show a lot of unreadable data in the terminal. If you don't receive any message, it probably means that gazebo is not publishing the camera data.

Check that the clock is being published by Gazebo:

```bash
rostopic echo /clock
```

If it is not, you have a problem with Gazebo (Did it finish loading the world? Do you see the buildings and the drone in the Gazebo UI?). However, if it is publishing the clock, then it might be a problem with the depth camera plugin. Make sure the package `ros-kinetic-gazebo-ros-pkgs` is installed. If not, install it and rebuild the Firmware (with `$ make posix_sitl_default gazebo` as explained above).

### I see the drone and world in rviz, but the drone does not move when I set a new "2D Nav Goal"
Is the drone in OFFBOARD mode? Is it armed and flying?

```bash
# Set the drone to OFFBOARD mode
rosrun mavros mavsys mode -c OFFBOARD
# Arm
rosrun mavros mavsafety arm
```

### I see the drone and world in rviz, but the drone does not follow the path properly
Some tuning may be required in the file *"<Firmware_dir>/posix-configs/SITL/init/rcS_gazebo_iris"*.

### I see the drone and world in rviz, I am in OFFBOARD mode, but the planner is still not working
Some parameters that can be tuned in *rqt reconfigure*.

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

The planner can be built with:

```bash
catkin build
```

Note that you can build it in release mode by adding `--cmake-args -DCMAKE_BUILD_TYPE=Release` as an argument to catkin:

```bash
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

And then just run with the corresponding launch file:

```bash
roslaunch global_planner global_planner_offboard.launch point_cloud_topic:=<point_cloud_topic>
```

# Running on Odroid

Read the [Running on Odroid](https://github.com/PX4/avoidance/tree/master/global_planner/resource/odroid) instructions.
