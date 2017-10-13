# Running development containers

First, build the `mavros` image, as `mavros-avoidance` depends on it. This step should be removed in the future by having the mavros image on the Docker hub (see [here](https://github.com/PX4/containers/issues/75). From the [components](../../components) directory, run:

    $ docker-compose -f components.yml build mavros

You need to open access to your X server:

    $ xhost +local:root

Running the full simulation can be done by running both `avoidance-node` and `sitl-avoidance-gui`, which will in turn run `mavros-avoidance` and `sitl-avoidance-server` as dependencies.

    $ docker-compose up avoidance-node sitl-avoidance-gui

If you don't need to run the GUI for your development, just omit it:

    $ docker-compose up avoidance-node

The ROS master is run by `mavros-avoidance`, which binds port 11311 of the container to 11311 on the host. This means that your host will automatically connect to the docker ROS system. In case of troubles, you might have to set `ROS_IP` on your host, but it should not even be necessary.

Stopping all the containers can be done with docker-compose, too:

    $ docker-compose down
