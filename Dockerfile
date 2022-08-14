FROM ros:noetic

# copy selected packages to desired directory in container
COPY . /root/catkin_ws/src/rviz_rotatable_image_plugin

SHELL ["bash", "-c"]

# install dependencies
RUN apt update && \
  apt install python3-catkin-tools -y && \
  cd /root/catkin_ws && \
  rosdep update && \
  rosdep install --from-paths src --ignore-src -r -y

# build repo
RUN . /ros_entrypoint.sh && cd /root/catkin_ws && \
  catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  sed -i '$isource "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
