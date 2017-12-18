FROM ros:kinetic-ros-base

RUN apt-get update && \
    apt-get install -y ros-kinetic-mavros

RUN /opt/ros/${ROS_DISTRO}/lib/mavros/install_geographiclib_datasets.sh

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["fcu_url:=udp://:14540@127.0.0.1:14557"]
