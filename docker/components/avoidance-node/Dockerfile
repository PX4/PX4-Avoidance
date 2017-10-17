FROM ros:kinetic-ros-base

RUN apt-get update && \
    apt-get install -y libopencv-dev \
                       python-catkin-tools \
                       ros-kinetic-octomap \
                       ros-kinetic-octomap-mapping

COPY node /root/catkin_ws/src/avoidance
COPY docker/components/avoidance-node/launch /root/launch

COPY docker/components/avoidance-node/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

RUN ["/bin/bash", "-c", "\
    source /opt/ros/kinetic/setup.bash && \
    catkin build -w /root/catkin_ws \
"]

ENV ROS_MASTER_URI http://mavros-avoidance:11311

ENTRYPOINT ["/root/entrypoint.sh"]
CMD global_avoidance.launch use_sim_time:="false"
