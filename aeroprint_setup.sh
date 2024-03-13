#!/bin/sh
sudo apt-get install git -y
if cd aeroprint; then git pull; else
    git clone git@github.com:kuederleR/aeroprint.git;
fi
if [[ $(git config --get user.name) == true ]]; then
   echo "Git already configured";
else
   read -p "Enter your name: " name
   read -p "Enter your email: " email

   git config --global user.name "$name"
   git config --global user.email "$email"
   git config --global push.default simple
   git config --global core.ignorecase false
   git config --global core.autocrlf input
   echo "git configured sucessfully."
fi
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

