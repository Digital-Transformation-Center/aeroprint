# Use a ROS Foxy base image for compatibility with your Starling's Foxy installation
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Define user arguments
ARG USERNAME=aeroprint_user
ARG USER_UID=1000
ARG USER_GID=1000

# Create user and grant sudo permissions without password (adjust if not desired)
# Using sudo to add to sudoers and then removing sudo from the path for security
RUN apt-get update && \
    apt-get install -y sudo git python3-pip && \
    groupadd --gid $USER_GID $USERNAME ; \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    chmod 0440 /etc/sudoers && \
    rm -rf /var/lib/apt/lists/*

# Switch to the new user
USER $USERNAME
WORKDIR /home/$USERNAME

# Create ROS 2 workspace
RUN mkdir -p external_packages/src

# --- Install px4_msgs from source compatible with ROS Foxy ---
# Important: Choose the correct branch for PX4_msgs that is compatible with ROS Foxy and your PX4 firmware
# Refer to https://github.com/PX4/px4_msgs and check their compatibility table.
# For ROS Foxy, 'release/1.13' or 'release/1.14' or 'main' depending on your PX4 version
# Let's assume 'main' or a specific release branch for Foxy
ARG PX4_MSGS_BRANCH="release/1.14"

# Clone px4_msgs into the workspace src directory
RUN git clone --branch ${PX4_MSGS_BRANCH} https://github.com/PX4/px4_msgs.git external_packages/src/px4_msgs

# Install dependencies (including those for px4_msgs) and build the workspace
# Using SHELL for proper sourcing during the build command
SHELL ["/bin/bash", "-c"]

USER root

# Install common ROS 2 tools and python requirements
COPY --chown=$USERNAME:$USERNAME exec/requirements.txt /tmp/requirements.txt
RUN sudo apt-get update && \
    sudo apt-get install -y \
    # ros-foxy-ros-base \
    # ros-foxy-rqt* \
    # python3-colcon-common-extensions \
    iputils-ping && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt && \
    rm -rf /var/lib/apt/lists/* /tmp/requirements.txt

# Build the workspace (including px4_msgs)
# Ensure ROS environment is sourced before building
WORKDIR /home/$USERNAME/external_packages

USER $USERNAME
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select px4_msgs && \
    rm -rf external_packages/log external_packages/build

# Copy your custom ROS 2 source code into the workspace
# Make sure your custom packages also declare px4_msgs as a dependency in their package.xml
# COPY --chown=$USERNAME:$USERNAME ../../src /home/$USERNAME/ros2_ws/src/your_custom_packages
# RUN mkdir -p /home/$USERNAME/aeroprint/src
WORKDIR /home/$USERNAME/aeroprint/

RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    source /home/$USERNAME/external_packages/install/setup.bash && \
    colcon build
# Ensure both ROS and workspace overlays are sourced in .bashrc
# This makes them available when you exec into the container
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/external_packages/install/setup.bash" >> /home/$USERNAME/.bashrc

# Copy entrypoint.sh and set permissions
COPY --chown=$USERNAME:$USERNAME exec/entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

USER root

# Set the default command for the container
# ENTRYPOINT sets the primary command. CMD provides default arguments.
# If you just want to run the entrypoint, you can simplify.
# "/usr/bin/env bash" is often more robust than just "bash" for the shell.
ENTRYPOINT ["/usr/bin/env", "bash", "-c", "/home/aeroprint_user/entrypoint.sh; exec bash"]
CMD [""] 