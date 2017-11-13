FROM components_mavros

COPY docker/components/mavros-avoidance/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

COPY global_planner/resource/px4_config.yaml /root/catkin_ws/src/mavros/mavros/launch/px4_config.yaml
