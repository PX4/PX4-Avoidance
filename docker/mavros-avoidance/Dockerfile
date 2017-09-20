FROM docker_mavros

COPY docker/mavros-avoidance/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

COPY node/resource/px4_config.yaml /root/catkin_ws/src/mavros/mavros/launch/px4_config.yaml
