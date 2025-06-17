# Use the ROS Humble ros-core base image, which includes Python 3 and core ROS 2 packages.
FROM ros:humble-ros-core

# Set locale to avoid warnings and ensure proper character encoding.
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Set up a non-root user for better security practices.
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID rosuser && useradd -m -s /bin/bash -u $UID -g $GID rosuser
USER rosuser
WORKDIR /home/rosuser

# Set the entrypoint to directly run the ROS 2 Python talker node.
# This sources the ROS environment and then executes the talker node immediately.
ENTRYPOINT ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && ros2 run demo_nodes_py talker"]
