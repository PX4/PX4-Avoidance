# Avoidance with Docker

## Running the demo

### Prerequisites

[Install docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/) and docker-compose:

    $ apt install docker-compose

### Run the demo

First, you need to open access to your X server for Docker, which might have (not necessarily big) security implications (more on this [here](http://wiki.ros.org/docker/Tutorials/GUI):

    $ xhost +local:root

Run the demo with docker-compose, from this directory ("docker" directory in the avoidance repository):

    $ docker-compose up ubuntu-avoidance

The first time, docker-compose will have to build the docker image, which will take some time.

Once you are done with it, you can stop the demo with "Ctrl + C" and possibly remove the X permissions you gave before:

    $ xhost -local:root

## Developing with Docker

In order to develop in good conditions, it is always nice to compile the code on the host. But installing the whole simulation on your host requires that you install everything, including Gazebo and a whole lot of ROS packages. Setting up the right environment might be troublesome if you happen to not be developing on Ubuntu 16.04. Turns out that if you only work on the avoidance node, maybe you don't need to modify anything in Gazebo or Mavros. In this case, you could just run Gazebo and Mavros on docker containers, and connect your avoidance node directly from the host. For this reason, different containers have been created:

* `sitl-avoidance-server`, in *sitl-server*, is running gazebo in server mode.
* `sitl-avoidance-gui`, in *sitl-gui*, runs gazebo and rviz.
* `mavros-avoidance` is based on `mavros` and obviously runs mavros.
* `avoidance-node` runs the actual avoidance system, composed of different nodes (including the octomap one).

First, build the `mavros` image, as `mavros-avoidance` depends on it. The goal is to have the mavros image on the Docker hub in the future.

    $ docker-compose build mavros

As described [above](#run-the-demo), you need to open access to your X server:

    $ xhost +local:root

Running the full simulation can be done by running both `avoidance-node` and `sitl-avoidance-gui`, which will in turn run `mavros-avoidance` and `sitl-avoidance-server` because they depend on them:

    $ docker-compose up avoidance-node sitl-avoidance-gui

If you don't need to run the GUI for your development, just omit it:

    $ docker-compose up avoidance-node

Once the avoidance node is setup on your host, just run the other modules. If you want the GUI:

    $ docker-compose up sitl-avoidance-gui

If you don't need the GUI, just run the server:

    $ docker-compose up sitl-avoidance-server

The ROS master is run by `mavros-avoidance`, which binds port 11311 of the container to 11311 on the host. This means that your host will automatically connect to the docker ROS system. In case of troubles, you might have to set `ROS_IP` on your host, but it should not even be necessary.

Stopping all the containers can be done with docker-compose, too:

    $ docker-compose down
