FROM ros:kinetic-ros-core

# Installation instructions: https://dev.px4.io/en/ros/mavros_installation.html

RUN apt-get update && \
    apt-get install -y python-wstool \
                       python-rosinstall-generator \
                       python-catkin-tools && \
    mkdir -p ~/catkin_ws/src && \
    catkin init --workspace ~/catkin_ws && \
    wstool init ~/catkin_ws/src && \
    rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall && \
    rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall && \
    wstool merge -t ~/catkin_ws/src /tmp/mavros.rosinstall && \
    wstool update -t ~/catkin_ws/src && \
    rosdep install --from-paths ~/catkin_ws/src --ignore-src --rosdistro kinetic -y

RUN ["/bin/bash", "-c", "source /opt/ros/kinetic/setup.bash && catkin build --workspace ~/catkin_ws"]

RUN ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["fcu_url:=\"udp://:14540@127.0.0.1:14557\""]
