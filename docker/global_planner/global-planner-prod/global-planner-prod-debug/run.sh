#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

docker-compose -f ${DIR}/docker-compose.yml up -d || { exit 1; }

VPN_SERVER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_alpinevpn_1)
ROS_MASTER_URI=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' globalplannerproddebug_mavros-avoidance_1)
ROUTE_IP=$(echo $ROS_MASTER_URI | sed 's/[[:digit:]]\+\.[[:digit:]]\+$/0.0/')

route add -net 10.0.0.0/16 gw $VPN_SERVER_IP

echo "The system has been setup in debug mode, with a vpn server allowing you to connect to the ROS master from a remote machine. To do so, follow these instructions from the remote machine:

1. Connect to the VPN from one terminal:

    $ sudo openvpn --remote 192.168.7.2 --dev tun --ifconfig 10.0.0.2 10.0.0.1 --route ${ROUTE_IP} 255.255.255.0

2. In another terminal (say the \"ROS terminal\"), setup the ROS environment variables like so:

    $ export ROS_MASTER_URI=http://${ROS_MASTER_URI}:11311
    $ export ROS_IP=10.0.0.2

3. From the ROS terminal, you can now run Rviz or check the topics with commands such as:

    $ rostopic list

Note that this assume the remote machine connects to the drone on 192.168.7.2 (which is the USB connection on the Intel Aero). This can be adapted to your needs.

Press any key to continue..."

read

docker-compose -f ${DIR}/docker-compose.yml logs -f
