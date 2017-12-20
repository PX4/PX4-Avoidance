FROM ros:kinetic-ros-base

ENV WORKSPACE_DIR /root
ENV CATKIN_WS ${WORKSPACE_DIR}/catkin_ws
ENV AVOIDANCE_DIR ${CATKIN_WS}/src/global-planner

RUN apt-get update && \
    apt-get install -y python-catkin-tools \
                       ros-kinetic-octomap \
                       ros-kinetic-octomap-mapping

RUN mkdir -p ${CATKIN_WS}/src

COPY global_planner ${AVOIDANCE_DIR}
COPY docker/components/global-planner-node/launch ${WORKSPACE_DIR}/launch

COPY docker/components/global-planner-node/entrypoint.sh ${WORKSPACE_DIR}/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

RUN ["/bin/bash", "-c", "\
    source /opt/ros/kinetic/setup.bash && \
    catkin build -w /root/catkin_ws --cmake-args -DCMAKE_BUILD_TYPE=Release \
"]

ENV ROS_MASTER_URI http://mavros-avoidance:11311

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["global_avoidance.launch use_sim_time:=\"false\""]
