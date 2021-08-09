
# Obstacle Detection and Avoidance
[![Release Status](https://img.shields.io/github/v/release/PX4/avoidance)](https://github.com/PX4/avoidance/releases)
[![Build Status](https://travis-ci.org/PX4/avoidance.svg?branch=master)](https://travis-ci.org/PX4/avoidance)
[![Coverage Status](https://coveralls.io/repos/github/PX4/avoidance/badge.svg)](https://coveralls.io/github/PX4/avoidance)

PX4 computer vision algorithms packaged as ROS nodes for depth sensor fusion and obstacle avoidance. This repository contains two different implementations:

  * *local_planner* is a local VFH+* based planner that plans (including some history) in a vector field histogram
  * *global_planner* is a global, graph based planner that plans in a traditional octomap occupancy grid
  * *safe_landing_planner* is a local planner to find safe area to land

The three algorithms are standalone and they are not meant to be used together.

The *local_planner* requires less computational power but it doesn't compute optimal paths towards the goal since it doesn't store information about the already explored environment. An in-depth discussion on how it works can be found in [this thesis](https://drive.google.com/open?id=1yjDtxRrIntr5Mdaj9CCB4IFJn0Iy2-bR). On the other hand, the *global_planner* is computationally more expensive since it builds a map of the environment. For the map to be good enough for navigation, accurate global position and heading are required. An in-depth discussion on how it works can be found in [this thesis](https://drive.google.com/open?id=1hhMLoXQuEM4ppdvDik8r6JY3RwGi5nzw).
The *safe_landing_planner* classifies the terrain underneath the vehicle based on the mean and standard deviation of the z coordinate of pointcloud points. The pointcloud from a downwards facing sensor is binned into a 2D grid based on the xy point coordinates. For each bin, the mean and standard deviation of z coordinate of the points are calculated and they are used to locate flat areas where it is safe to land.

> **Note** The development team is right now focused on the *local_planner*.

The documentation contains information about how to setup and run the two planner systems on the Gazebo simulator and on a companion computer running Ubuntu 16.04, for both avoidance and collision prevention use cases.

> **Note** PX4-side setup is covered in the PX4 User Guide:
  - [Obstacle Avoidance](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html)
  - [Collision Prevention](https://docs.px4.io/en/computer_vision/collision_prevention.html)

[![PX4 Avoidance video](http://img.youtube.com/vi/VqZkAWSl_U0/0.jpg)](https://www.youtube.com/watch?v=VqZkAWSl_U0)

# Table of Contents
- [Getting Started](#getting-started)
  - [Installation](#installation)
    - [Installation for Ubuntu](#installation)
  - [Run the Avoidance Gazebo Simulation](#run-the-avoidance-gazebosimulation)
    - [Local Planner](#local-planner)
    - [Global Planner](#global-planner)
    - [Safe Landing Planner](#safe-landing-planner)
  - [Run on Hardware](#run-on-hardware)
    - [Prerequisite](#prerequisite)
    - [Local Planner](#local-planner)
    - [Global Planner](#global-planner)
- [Troubleshooting](#troubleshooting)
- [Advanced](#advanced)
  - [Message Flows](#message-flow)
    - [PX4 and local planner](#px4-and-local-planner)
    - [PX4 and global planner](#px4-and-gloabl-planner)
- [Contributing](#contributing)

# Getting Started

## Installation

### Installation

This is a step-by-step guide to install and build all the prerequisites for running the avoidance module on either:
- **Ubuntu 20.04:** *ROS2 Foxy* with Gazebo 11 (preferred).
You might want to skip some steps if your system is already partially installed.

> **Note:** These instructions assume your workspace (in which we will build the avoidance module) is in `~/avoidance_ws/src/`, and the PX4 Firmware directory is `~/Firmware`.
  Feel free to adapt this to your situation.

1. Install ROS foxy to your PC. Follow steps on [#Installing ROS 2 via Debian Packages](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Install-Debians/)

1. Clone this repository in your avoidance workspace in order to build the avoidance node (ros2 branch) and px4_ros_com to use micrortps_agent

   ```bash
   cd ~/avoidance_ws/src/
   git clone -b ros2 --recursive https://github.com/PX4/avoidance.git
   git clone https://github.com/PX4/px4_ros_com.git
   ```

1. Actually build the avoidance node. (exclude local_planner since it is not fully working at this time)

   ```bash
   cd ~/avoidance_ws
   colcon build --packages-skip local_planner
   ```


1. Source the workspace setup.bash from your avoidance workspace:
   ```bash   
   echo "source ~/avoidance_ws/src/PX4-Avoidance/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Run the Avoidance Gazebo Simulation

In the following section we guide you through installing and running a Gazebo simulation of both local and global planner.

### Build the Simulator

1. Clone the PX4 Firmware and all its submodules (it may take some time).

   ```bash
   cd ~
   git clone https://github.com/PX4/Firmware.git --recursive
   cd ~/Firmware
   ```

1. Install [PX4 dependencies](http://dev.px4.io/en/setup/dev_env_linux_ubuntu.html#common-dependencies). 
   ```bash
   # Install PX4 "common" dependencies.
   ./Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
   
   # Gstreamer plugins (for Gazebo camera)
   sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly libgstreamer-plugins-base1.0-dev

1. Build the Firmware once in order to generate SDF model files for Gazebo.
   This step will actually run a simulation (that you can immediately close).

   ```bash
   # This is necessary to prevent some Qt-related errors (feel free to try to omit it)
   export QT_X11_NO_MITSHM=1

   # Build and run simulation
   make px4_sitl_rtps gazebo
   
   # Quit the simulation (Ctrl+C)

   # Setup some more Gazebo-related environment variables (modify this line based on the location of the Firmware folder on your machine)
   . ~/Firmware/Tools/setup_gazebo.bash ~/Firmware ~/Firmware/build/px4_sitl_rtps
   ```

1. Set the GAZEBO_MODEL_PATH in your bashrc:
   ```bash
   echo "export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/avoidance_ws/src/PX4-avoidance/avoidance/sim/models:~/avoidance_ws/src/PX4-avoidance/avoidance/sim/worlds" >> ~/.bashrc
   ```

The last three steps, together with sourcing your ros2 workspace **setup.bash** (`source ~/avoidance_ws/install/setup.bash`) should be repeated each time a new terminal window is open.
You should now be ready to run the simulation using local or global planner.

### Run gazebo simulator for using planners

1. Run px4_sitl_rtps
 ```bash
   # Build and run simulation
   make px4_sitl_rtps gazebo
   ```

2. Run px4_ros_com to send/receive msgs with drone
```bash
   ~/avoidance_ws/build/px4_ros_com/micrortps_agent -t UDP
```

3.  Check communication via ros2 has no problem
```bash
   # See which topics are existing
   ros2 topic list

   # Echo topic data for test
   ros2 topic echo /VehicleLocalPosition_PubSubTopic
```

### Local Planner (default, ros2 version is not ready)

This section shows how to start the *local_planner* and use it for avoidance in mission or offboard mode.

The planner is based on the [3DVFH+](http://ceur-ws.org/Vol-1319/morse14_paper_08.pdf) algorithm.

> **Note:** You *may* need to install some additional dependencies to run the following code (if not installed):
> * Foxy:
>   ```sh
>   sudo apt install ros-foxy-stereo-image-proc ros-foxy-image-view
>   ```

** Documentation for local planner in ros2 environment should be written here. **


### Global Planner (advanced, simulation tested)

This section shows how to start the *global_planner* and use it for avoidance in auto_loiter mode.

```bash
ros2 launch global_planner global_planner_depth_camera.launch.py
```

You should now see the drone unarmed on the ground in a forest environment as pictured below. (gazebo and rviz2)
![Screenshot showing gazebo](docs/gazebo_simple_obstacle_screenshot.png)

![Screenshot showing rviz](docs/simulation_screenshot.png)

To start flying, arm it. The avoidance node will then take control of it.
You may use QGroundControl to arm the drone.

```bash
# In another terminal
ros2 topic pub --once /VehicleCommand_PubSubTopic px4_msgs/msg/VehicleCommand "{target_system: 1, command: 400, param1: 1.0, from_external: true}"
```

Initially the drone should just hover at 3.5m altitude.

From the command line, you can also make Gazebo follow the drone, if you want.

```bash
gz camera --camera-name=gzclient_camera --follow=iris
```

One can plan a new path by setting a new goal with the *2D Nav Goal* button in rviz.
The planned path should show up in rviz and the drone should follow the path, updating it when obstacles are detected.
It is also possible to set a goal without using the obstacle avoidance (i.e. the drone will go straight to this goal and potentially collide with obstacles). To do so, set the position with the *2D Pose Estimate* button in rviz.


### Safe Landing Planner (ros2 version is not ready)

This section shows how to start the *safe_landing_planner* and use it to land safely in mission or auto land mode. To run the node:

```bash
ros2 launch safe_landing_planner safe_landing_planner.launch.py
```

You will see an unarmed vehicle on the ground. Open [QGroundControl](http://qgroundcontrol.com/), either plan a mission with the last item of type *Land* or fly around the world in Position Control, click the *Land* button on the left side where you wish to land.
At the land position, the vehicle will start to descend towards the ground until it is at `loiter_height` from the ground/obstacle. Then it will start loitering to evaluate the ground underneeth.
If the ground is flat, the vehicle will continue landing. Otherwise it will evaluate the close by terrain in a squared spiral pattern until it finds a good enough ground to land on.

# Run on Hardware

Testing avoidance nodes in ros2 environment is not yet done.
We need support to test nodes with ros2.

# Troubleshooting

Troubleshooting for ros2 environment is not yet written.

# Advanced

## Message Flows

More information about the communication between avoidance system and the Autopilot can be found in the [PX4 User Guide](https://docs.px4.io/en/computer_vision/obstacle_avoidance.html)

### PX4 and local planner

Message flow for ros2 environment is not yet written.


### PX4 and global planner

This is the complete message flow *from* PX4 Firmware *to* the global planner.

PX4 topic | MAVLink | ROS Msgs. | Topic
--- | --- | --- | ---
vehicle_attitude | ATTITUDE | px4_msgs::msg::VehicleAttitude | /VehicleAttitude_PubSubTopic
vehicle_global_position | GLOBAL_POSITION_INT | px4_msgs::msg::VehicleGlobalPosition | /VehicleGlobalPosition_PubSubTopic
vehicle_local_position | LOCAL_POSITION_NED | px4_msgs::msg::VehicleLocalPosition | /VehicleLocalPosition_PubSubTopic

This is the complete message flow *to* PX4 Firmware *from* the global planner.

ROS topic | ROS Msgs. | MAVLink | PX4 Topic
--- | --- | --- | ---
/VehicleCommand_PubSubTopic | px4_msgs::msg::VehicleCommand | VEHICLE_COMMAND | vehicle_command

### PX4 and safe landing planner

Message flow for ros2 environment is not yet written.


# Contributing

Fork the project and then clone your repository. Create a new branch off of master for your new feature or bug fix.

Please, take into consideration our [coding style](https://github.com/PX4/avoidance/blob/master/tools/fix_style.sh).
For convenience, you can install the commit hooks which will run this formatting on every commit. To do so, run
`./tools/set_up_commit_hooks` from the main directory.

Commit your changes with informative commit messages, push your branch and open a new pull request. Please provide ROS bags and the Autopilot flight logs relevant to the changes you have made.
