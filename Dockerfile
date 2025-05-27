# Use the official ROS2 Humble base image
FROM ros:foxy

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    git \
    ros-foxy-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

# Copy pre-built px4_msgs package into the container
# COPY px4_msgs_install/ /px4_msgs/install/

# Copy the workspace into the container
COPY ./ /workspace/

# Set the working directory to the workspace
WORKDIR /workspace

# Build the workspace
# RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && colcon build"

RUN echo "source /opt/ros/foxy/setup.bash" >> /root/.bashrc

# Run your application


# Set the entrypoint to the bash shell
# CMD ["/bin/bash", "-c", "source /opt/ros/foxy/setup.bash"]
CMD ["/bin/bash"]