# Deployment

1. First, make sure Docker and docker-compose are installed.
2. Checkout the avoidance repository on the target drone/machine.
3. From the [components](../../../components) folder, run:

    $ docker-compose -f components.yml build mavros

4. From this folder ("global-planner-prod-release"), run:

    $ docker-compose up

The realsense, mavros and avoidance containers will get built (this can take some time) and run.
