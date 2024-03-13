#!/bin/sh

sudo apt-get update && sudo apt-get upgrade -y
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-foxy-desktop python3-argcomplete -y
sudo apt install python3-colcon-common-extensions -y
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source aeroprint/install/local_setup.bash" >> ~/.bashrc

