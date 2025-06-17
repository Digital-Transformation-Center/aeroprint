FROM ros:humble

# Install demo_nodes_py package
RUN apt-get update && \
    apt-get install -y ros-humble-demo-nodes-py && \
    rm -rf /var/lib/apt/lists/*

# Set up ROS 2 environment
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]

# Run the talker node
CMD ["ros2", "run", "demo_nodes_py", "talker"]