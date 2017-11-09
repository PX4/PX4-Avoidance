FROM components_sitl-avoidance-server

COPY global_planner/resource/global_planner.rviz /root/resource/global_planner.rviz
COPY local_planner/resource/local_planner.rviz /root/resource/local_planner.rviz

COPY docker/components/sitl-gui/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
