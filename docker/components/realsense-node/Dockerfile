FROM ros:kinetic-ros-base

RUN apt-get update && \
    apt-get install -y ros-kinetic-realsense-camera

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT /root/entrypoint.sh
