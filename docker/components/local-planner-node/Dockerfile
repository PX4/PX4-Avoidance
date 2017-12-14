FROM ros:kinetic-ros-base

ENV WORKSPACE_DIR /root
ENV CATKIN_WS ${WORKSPACE_DIR}/catkin_ws
ENV AVOIDANCE_DIR ${CATKIN_WS}/src/local-planner

RUN apt-get update && \
    apt-get install -y libopencv-dev \
                       python-catkin-tools \
                       ros-kinetic-mavros-msgs \
                       ros-kinetic-pcl-ros

COPY local_planner ${AVOIDANCE_DIR}
COPY docker/components/local-planner-node/launch ${WORKSPACE_DIR}/launch

COPY docker/components/local-planner-node/entrypoint.sh ${WORKSPACE_DIR}/entrypoint.sh
RUN chmod +x ${WORKSPACE_DIR}/entrypoint.sh

RUN ["/bin/bash", "-c", "\
    source /opt/ros/kinetic/setup.bash && \
    catkin build -w ${CATKIN_WS} --cmake-args -DCMAKE_BUILD_TYPE=Release \
"]

ENV ROS_MASTER_URI http://mavros-avoidance:11311

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["local_avoidance.launch use_sim_time:=\"false\""]
