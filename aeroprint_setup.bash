#!/bin/bash

RED='\033[0;31m'
GREEN='\033[32m'

NC='\033[0m' # No Color
echo -e "${RED}Setting up the AeroPrint tools.${NC}"

## Install or update git
sudo apt-get install git -y
if cd ~/aeroprint; then echo -e "${GREEN}Updating the repository.${NC}" && git pull; else
    echo -e "${RED}Repository does not exist on this system.${NC}"
    echo -e "${GREEN}Cloning repository.${NC}"
    echo "Cloning repository."
    git clone git@github.com:kuederleR/aeroprint.git
fi

## Install ROS2 Foxy
if printenv ROS_DISTRO | grep foxy -q; then echo -e "${GREEN}Ros 2 Foxy already installed.${NC}"; else
    echo -e "${GREEN}Installing ROS 2 Foxy.${NC}"
    sudo apt-get update && sudo apt-get upgrade -y
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update && sudo apt upgrade -y
    sudo apt install ros-foxy-desktop python3-argcomplete -y
    sudo apt install python3-colcon-common-extensions -y
fi

## Install RosDep
if rosdep --version > /dev/null; then echo -e "${GREEN}RosDep already installed.${NC}"; else
    echo -e "${GREEN}Installing RosDep.${NC}"
    sudo apt-get install python3-rosdep
fi

## Source ROS2 Foxy
if cat ~/.bashrc | grep "source /opt/ros/foxy/setup.bash" -q; then echo -e "${GREEN}ROS 2 Foxy already sourced.${NC}"; else
    echo -e "${GREEN}Sourcing ROS 2 Foxy.${NC}"
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
fi

## Source aeroprint package by default
if cat ~/.bashrc | grep "source ~/aeroprint/install/setup.bash" -q; then echo -e "${GREEN}AeroPrint package already sourced.${NC}"; else
    echo -e "${GREEN}Sourcing AeroPrint setup.${NC}"
    echo "source ~/aeroprint/install/setup.bash" >> ~/.bashrc
fi

## Install pip
if pip3 --version > /dev/null; then echo -e "${GREEN}Pip already installed.${NC}"; else
    echo -e "${GREEN}Installing pip.${NC}"
    sudo apt-get install python3-pip
fi

