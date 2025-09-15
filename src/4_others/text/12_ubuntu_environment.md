# Ubuntu Environment

This guide provides step-by-step instructions for setting up the system, installing essential tools, and configuring the environment for ROS2.

```bash
sudo apt update
sudo apt upgrade

#swap
sudo fallocate -l 16G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
sudo swapon --show


# basic installs
sudo apt install vim git htop cmake build-essential clang net-tools -y openssh-server mc tree

# ROS2
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

sudo snap install code --classic
sudo snap install clion --classic
sudo snap install pycharm-community --classic
sudo snap install rustrover --classic

sudo apt update
sudo apt install ros-humble-image-transport-plugins -y

```
