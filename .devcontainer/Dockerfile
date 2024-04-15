FROM ros:foxy
ARG USERNAME=ap_dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV SHELL /bin/bash
# Create the user and update permissions
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0700 /etc/sudoers.d/$USERNAME
# Update
RUN apt-get update && apt-get upgrade -y
# Install necessary packages
RUN apt-get install -y python3-pip \ 
    && apt-get install python3-colcon-common-extensions -y \
    && apt-get install python3-rosdep -y \
    && echo "source /opt/ros/foxy/setup.bash" >> /etc/bash.bashrc \
    && rosdep update

USER $USERNAME
CMD ["/bin/bash"]