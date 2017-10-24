#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

docker-compose -f ${DIR}/docker-compose.yml up -d

VPN_SERVER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_alpinevpn_1)
ROS_MASTER_URI=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_mavros-avoidance_1)

route add -net 10.0.0.0/16 gw $VPN_SERVER_IP

echo "The system has been setup in debug mode, with a vpn server allowing you to connect to the ROS master from a remote machine. To do so:

1. From the remote machine, run:

    $ export ROS_MASTER_URI=http://${ROS_MASTER_URI}:11311
    $ export ROS_IP=10.0.0.2
    $ sudo openvpn --remote 192.168.7.2 --dev tun --ifconfig 10.0.0.2 10.0.0.1 --route 172.20.0.0 255.255.255.0

Note that this assume the remote machine connects to the drone on 192.168.7.2 (which is the USB connection on the Intel Aero). This can be adapted to your needs.

Press any key to continue..."

read

docker-compose -f ${DIR}/docker-compose.yml logs -f
