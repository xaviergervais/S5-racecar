#!/bin/bash

# set distro
export ROS_DISTRO=humble

#################### install utils ############################################

sudo apt update
sudo apt install net-tools nmap htop -y

#################### install ROS ############################################

# locale  # check for UTF-8
sudo apt update && sudo apt install locales -y 
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale  # verify settings

sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install ros-dev-tools -y
sudo apt update && sud apt upgrade -y
sudo apt install ros-${ROS_DISTRO}-desktop -y

# Source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

#################### setup repos ############################################

# Dependencies
cd ~
mkdir -p ~/racecar_dep/src && cd ~/racecar_dep/src
git clone --depth 1 --branch ros2-develop https://github.com/fkie/async_web_server_cpp.git
git clone --depth 1 --branch humble https://github.com/rst-tu-dortmund/costmap_converter.git
git clone --depth 1 --branch ros2 https://github.com/babakhani/rplidar_ros2.git
git clone --depth 1 --branch ros2 https://github.com/chameau5050/web_video_server.git
cd ~/racecar_dep
sudo rosdep init
rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -y

# Build dependencies (N.B. Ça prend du temps)
cd ~/racecar_dep
colcon build --symlink-install
source install/setup.bash

echo "source ~/racecar_dep/install/setup.bash" >> ~/.bashrc

# racecar's workspace
cd ~
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone --depth 10 --branch ros2 https://github.com/SherbyRobotics/racecar.git
git clone --depth 1 --branch ros2-master https://github.com/rst-tu-dortmund/teb_local_planner.git
cd teb_local_planner
git checkout 630a22e88dc9fd45be726a762edbb5b776bef231
cd ~/ros2_ws
rosdep update
rosdep install --rosdistro=${ROS_DISTRO} --from-paths src --ignore-src -y

# "Just in case" installs
sudo apt install ros-${ROS_DISTRO}-navigation2 -y
sudo apt install ros-${ROS_DISTRO}-v4l2-camera -y 
sudo apt install ros-${ROS_DISTRO}-rtabmap-ros -y

source /opt/ros/${ROS_DISTRO}/setup.bash
source ~/racecar_dep/install/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc