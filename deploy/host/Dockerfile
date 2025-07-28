FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

ARG USERNAME=aeroprint_user
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME ; \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    usermod -aG sudo $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Create ROS 2 workspace and copy src
# RUN mkdir -p /home/$USERNAME/ros2_ws/src
# COPY --chown=$USERNAME:$USERNAME ./src /home/$USERNAME/ros2_ws/src

# Copy prebuilt px4_msgs
COPY px4_msgs_install.tar.gz /tmp/

RUN mkdir -p /opt/px4_msgs && \
    tar -xzf /tmp/px4_msgs_install.tar.gz -C /opt/px4_msgs

COPY requirements.txt /tmp/
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip3 install --no-cache-dir -r /tmp/requirements.txt && \
    rm /tmp/requirements.txt

# Ensure both ROS and px4_msgs overlays are sourced
RUN echo 'source /opt/px4_msgs/setup.bash' >> /home/$USERNAME/.bashrc

# USER $USERNAME
USER root

WORKDIR /home/$USERNAME

COPY --chown=aeroprint_user:aeroprint_user entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN chmod +x /home/$USERNAME/entrypoint.sh

# RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
#     echo "source /opt/px4_msgs/setup.bash" >> /home/$USERNAME/.bashrc

# COPY --chown=$USERNAME:$USERNAME ./px4_msgs/px4_msgs_install/ ./install/
# COPY --chown=$USERNAME:$USERNAME ./px4_msgs/px4_msgs_build/ ./build/

# RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && colcon build"

CMD ["/bin/bash"]
ENTRYPOINT ["/bin/bash", "-c", "/home/aeroprint_user/entrypoint.sh; exec bash"]