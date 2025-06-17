# Use an official ROS Humble base image.
# ros-base includes only the core ROS 2 packages, keeping the image size smaller.
FROM ros:humble-ros-base

# Set the locale to avoid warnings during ROS 2 setup.
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Set up a non-root user (optional but recommended for security)
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID rosuser && useradd -m -s /bin/bash -u $UID -g $GID rosuser
USER rosuser
WORKDIR /home/rosuser

# Create a ROS 2 workspace
WORKDIR /home/rosuser/ros2_ws
RUN mkdir -p src

# Clone the ros_tutorials repository which contains the rclcpp_tutorials talker node.
# This assumes Git is available in the base image, which it typically is.
# Using --depth 1 to reduce clone size.
RUN git clone --depth 1 https://github.com/ros/ros_tutorials.git src/ros_tutorials

# Install ROS dependencies for the workspace.
# Need to switch back to root temporarily for rosdep install.
USER root
RUN apt-get update && \
    # FIX: Remove the default rosdep sources list if it already exists, to prevent 'file already exists' error.
    rm -f /etc/ros/rosdep/sources.list.d/20-default.list && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths ros2_ws/src --ignore-src -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Switch back to the rosuser for building and running.
USER rosuser
WORKDIR /home/rosuser/ros2_ws

# Build the ROS 2 workspace.
# --symlink-install creates symlinks for executables, which is useful for development.
# --cmake-args -DCMAKE_BUILD_TYPE=Release for optimized builds.
RUN . /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source the ROS 2 setup file and the workspace setup file, then execute the talker node.
# Using ENTRYPOINT to ensure the ROS environment is always sourced.
# CMD provides default arguments to the ENTRYPOINT.
# This will run the talker node repeatedly.
ENTRYPOINT ["/bin/bash", "-c", ". /opt/ros/humble/setup.bash && . /home/rosuser/ros2_ws/install/setup.bash && ros2 run rclcpp_tutorials talker"]

# You can also use CMD if you prefer to override the command more easily when running the container:
# CMD ["ros2", "run", "rclcpp_tutorials", "talker"]
