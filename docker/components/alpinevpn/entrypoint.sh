#!/bin/ash

# Create tun interface if necessary
[ -d /dev/net ] || mkdir -p /dev/net
[ -c /dev/net/tun ] || mknod /dev/net/tun c 10 200

while true
do 
    openvpn --ifconfig 10.0.0.1 10.0.0.2 --dev tun --auth none
done >> /home/root/udp1194.log &
tail -F /home/root/udp1194.log
