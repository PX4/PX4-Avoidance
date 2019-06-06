FROM components_mavros

COPY docker/components/mavros-avoidance/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

COPY avoidance_global_planner/resource/px4_config.yaml /opt/ros/kinetic/share/mavros/launch/px4_config.yaml
