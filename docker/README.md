# Avoidance with Docker

## Prerequisites

[Install docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/) and docker-compose:

    $ apt install docker-compose

When using the X server in a docker container, you need to run the following command, which has (not necessarily big) security implications (more on this [here](http://wiki.ros.org/docker/Tutorials/GUI)):

    $ xhost +local:root

This can be undone with:

    $ xhost -local:root

## Quick Start

A wrapper script has been created to help running the planners in simulation or on the drone.

### Run the simulation

To run the simulation for the local planner, run:

```
$ ./run.py local
```

Similarly, for the global planner:

```
$ ./run.py global
```

### Run in production on the Aero

In production mode, the simulator is not started and the modules connect to the FCU on the Aero. There are two modes:

* __debug__: start a vpn server, allowing you to connect your machine into the ROS network on the Aero and e.g. run RViz on your machine.
* __release__: just start the production nodes.

To run the local planner in __debug__ mode:

```
$ ./run.py local --prod-debug
```

To run the local planner in __release__ mode:

```
$ ./run.py local --prod-release
```

To run the global planner in __debug__ mode:

```
$ ./run.py global --prod-debug
```

To run the global planner in __release__ mode:

```
$ ./run.py global --prod-release
```

## Running the demo

Find instructions [here](demo).

## Deploying with Docker

Deployment configurations are located in "global_planner/global-planner-prod".

The global planner can be deployed in two flavours: `debug` and `release`. They are currently the same, but in the future, the `debug` flavour will allow debugging from a remote machine (i.e. one will be able to run rviz on a computer while the containers are running on the drone).

Find instructions to run the `release` flavour [here](global_planner/global-planner-prod/global-planner-prod-release).

## Developing with Docker

Development configurations are located in "global_planner/global-planner-dev".

In order to develop in good conditions, it is always nice to compile the code on the host. But installing the whole simulation on your host requires that you install everything, including Gazebo and a whole lot of ROS packages. Setting up the right environment might be troublesome if you happen to not be developing on Ubuntu 16.04. Turns out that if you only work on the avoidance node, chances are that you don't need to modify anything in Gazebo or Mavros. In this case, you could just run Gazebo and Mavros on docker containers, and connect your avoidance node directly from the host. For this reason, different containers have been created (they are defined in [components](components)):

* `sitl-avoidance-server`, in *sitl-server*, is running gazebo in server mode (requires X server).
* `sitl-avoidance-gui`, in *sitl-gui*, runs gazebo and rviz (requires X server).
* `mavros-avoidance` is based on `mavros` and obviously runs mavros.
* `avoidance-node` runs the actual avoidance system, composed of different nodes (including the octomap one).

You will find [here](global_planner/global-planner-dev) instructions on how to run the global_planner containers in dev mode.
