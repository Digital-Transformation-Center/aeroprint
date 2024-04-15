FROM ros:foxy
ARG USERNAME=ap_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV SHELL /bin/bash
# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip \ 
    && apt install python3-colcon-common-extensions -y \
    && apt install python3-rosdep -y \
    && echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc

# ENTRYPOINT ["ros2", "/opt/ros/foxy/setup.bash"]
# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]