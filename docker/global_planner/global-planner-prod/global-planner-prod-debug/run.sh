#!/bin/bash

docker-compose up -d

VPN_SERVER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_alpinevpn_1)
ROS_MASTER_URI=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_mavros-avoidance_1)

route add -net 10.0.0.0/16 gw $VPN_SERVER_IP

echo "The system has been setup in debug mode, with a vpn server allowing you to connect to the ROS master from a remote machine. To do so:

1. From the remote machine, connect to the vpn, with "client.ovpn" the client config file. It is assuming a USB connection to the Aero, meaning the drone is available on 192.168.7.2. Edit the config file for a different setup.

    $ sudo openvpn --config client.ovpn

2. Set the MASTER_ROS_URI and ROS_IP on the remote machine:

    $ export ROS_MASTER_URI=http://${ROS_MASTER_URI}:11311
    $ export ROS_IP=<your vpn ip>

Press any key to continue..."

read

docker-compose logs -f
