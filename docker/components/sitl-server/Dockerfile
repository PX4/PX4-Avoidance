FROM osrf/ros:kinetic-desktop-full

ENV QT_X11_NO_MITSHM 1

ENV WORKSPACE_DIR /root
ENV FIRMWARE_DIR ${WORKSPACE_DIR}/Firmware
ENV SIM_DIR ${WORKSPACE_DIR}/sim
ENV MODELS_DIR ${SIM_DIR}/models
ENV WORLDS_DIR ${SIM_DIR}/worlds

RUN apt-get update && \
    apt-get install -y libopencv-dev \
                       python-jinja2 \
                       python-toml \
                       protobuf-compiler

RUN git clone https://github.com/PX4/Firmware.git ${FIRMWARE_DIR}
RUN git -C ${FIRMWARE_DIR} submodule update --init --recursive

COPY sim ${SIM_DIR}
COPY docker/components/sitl-server/px4_gazebo.launch ${WORKSPACE_DIR}
COPY docker/components/sitl-server/entrypoint.sh ${WORKSPACE_DIR}

RUN chmod +x ${WORKSPACE_DIR}/entrypoint.sh

ENV GAZEBO_MODEL_PATH ${GAZEBO_MODEL_PATH}:${MODELS_DIR}

RUN ["/bin/bash", "-c", " \
        cd ${FIRMWARE_DIR} && \
        . ${FIRMWARE_DIR}/Tools/setup_gazebo.bash ${FIRMWARE_DIR} ${FIRMWARE_DIR}/build/posix_sitl_default && \
        DONT_RUN=1 make posix_sitl_default gazebo \
"]

RUN echo "param set MAV_BROADCAST 1" >> ${FIRMWARE_DIR}/posix-configs/SITL/init/ekf2/iris

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["world_path:=\"/root/sim/worlds/test_city_2.world\""]
