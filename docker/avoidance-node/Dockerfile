FROM ros:kinetic-ros-base

RUN apt-get update && \
    apt-get install -y python-catkin-tools \
                       ros-kinetic-octomap \
                       ros-kinetic-octomap-mapping

COPY node /root/catkin_ws/src/avoidance
COPY docker/avoidance-node/cmakelists.patch /root/cmakelists.patch
RUN git -C /root/catkin_ws/src/avoidance apply /root/cmakelists.patch

COPY docker/avoidance-node/avoidance.launch /root/avoidance.launch

COPY docker/avoidance-node/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

RUN ["/bin/bash", "-c", "\
    source /opt/ros/kinetic/setup.bash && \
    catkin build -w /root/catkin_ws \
"]

ENV ROS_MASTER_URI http://mavros-avoidance:11311

ENTRYPOINT ["/root/entrypoint.sh"]
