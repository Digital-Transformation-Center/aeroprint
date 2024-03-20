# Use the official Ubuntu 20.04 base image
FROM ubuntu:20.04
ENV DEBIAN_FRONTEND=nonintercative
# Update package lists and install any necessary packages
RUN apt-get update && apt-get install -y \
    # Add your desired packages here (e.g., curl, vim, etc.)
    curl \
    vim \
    git \
    locales \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    add-apt-repository universe && \
    apt-get update && apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-foxy-desktop \
    python3-argcomplete && \
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# Set environment variables if needed
# ENV MY_ENV_VAR=value

# Start a shell when the container runs
CMD ["bash"]
