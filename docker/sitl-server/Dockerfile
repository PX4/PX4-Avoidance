FROM osrf/ros:kinetic-desktop-full

ENV QT_X11_NO_MITSHM 1

ENV WORKSPACE_DIR /root
ENV FIRMWARE_DIR ${WORKSPACE_DIR}/Firmware
ENV AVOIDANCE_DIR ${WORKSPACE_DIR}/avoidance

RUN apt-get update && \
    apt-get install -y libopencv-dev \
                       python-jinja2 \
                       protobuf-compiler

RUN git clone https://github.com/PX4/Firmware.git ${FIRMWARE_DIR}
RUN git -C ${FIRMWARE_DIR} submodule update --init --recursive

COPY node ${AVOIDANCE_DIR}
COPY docker/sitl-server/px4_gazebo.launch ${WORKSPACE_DIR}

ENV GAZEBO_MODEL_PATH ${GAZEBO_MODEL_PATH}:${AVOIDANCE_DIR}/models

RUN ["/bin/bash", "-c", " \
        cd ${FIRMWARE_DIR} && \
        . ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default && \
        DONT_RUN=1 make posix_sitl_default gazebo \
"]

RUN echo "param set MAV_BROADCAST 1" >> ${FIRMWARE_DIR}/posix-configs/SITL/init/ekf2/iris

CMD ["/bin/bash", "-c", " \
    source /opt/ros/kinetic/setup.bash && \
    export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:${FIRMWARE_DIR} && \
    . ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default && \
    export ROS_MASTER_URI=http://mavros-avoidance:11311 && \
    export ROS_IP=`hostname -I` && \
    roslaunch ${WORKSPACE_DIR}/px4_gazebo.launch gui:=false \
"]
