# OpenVPN for Docker on Alpine

This code is coming from https://github.com/besn0847/alpinevpn, itself coming from https://github.com/jpetazzo/dockvpn.

It is creating an OpenVPN server that opens the ROS network to external machines (for debugging purpose).

A notable modification made in this script is the IP address of the server.

## Running instructions

Build and run the container with docker-compose. From the [components](..) folder, run:

    $ docker-compose up alpinevpn

Once the container is running, one can copy the config file directly from the running container:

    $ docker cp <container_id>:/etc/openvpn/client.http .

Another way is to run an http server allowing to download the config file (follow the link given on the terminal to download the file):

    $ docker run -e VPN_SERVER_IP=<vpn_server_ip> -t -i -p 8080:8080 --volumes-from <container_id> <alpinevpn_image> serveconfig

In practice, it will probably be:

    $ docker run -e VPN_SERVER_IP=192.168.7.2 -t -i -p 8080:8080 --volumes-from globalplannerproddebug_alpinevpn_1 globalplannerproddebug_alpinevpn serveconfig

When you have the config file ("client.http") on the remote machine, fire up the openvpn client:

    $ openvpn --config client.http
