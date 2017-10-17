#!/bin/ash

# Create tun interface if necessary
[ -d /dev/net ] || mkdir -p /dev/net
[ -c /dev/net/tun ] || mknod /dev/net/tun c 10 200

# Set network mask and ip in server config file
NETWORK_IP=$(echo `hostname -i` | sed 's/\.\d\+$/.0/')
NETWORK_MASK="255.255.255.0"

sed -i "s/<NETWORK_IP>/$NETWORK_IP/g" /home/root/udp1194.conf
sed -i "s/<NETWORK_MASK>/$NETWORK_MASK/g" /home/root/udp1194.conf

iptables -t nat -A POSTROUTING -s 10.0.0.0/16 -o eth0 -j MASQUERADE

while true
do 
    openvpn /home/root/udp1194.conf
done >> /home/root/udp1194.log &
tail -F /home/root/udp1194.log
