# Running the demo

## Prerequisites

[Install docker](https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/) and docker-compose:

    $ apt install docker-compose

You will also need to open access to your X server for Docker, which might have (not necessarily big) security implications (more on this [here](http://wiki.ros.org/docker/Tutorials/GUI)):

    $ xhost +local:root

This command can be undone when you are finished with using the X server in Docker:

    $ xhost -local:root

### Run the demo

Run the demo with docker-compose, from this directory ("docker/demo" directory in the avoidance repository):

    $ docker-compose up

The first time, docker-compose will have to build the docker image, which will take some time.

Once you are done, you can stop the demo with "Ctrl + C" and possibly remove the X permissions you gave before:

    $ xhost -local:root
