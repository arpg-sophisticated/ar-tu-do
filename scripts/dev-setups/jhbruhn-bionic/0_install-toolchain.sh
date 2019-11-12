#!/usr/bin/env bash

# set -x

export ROS_DIST=melodic

# fix apt warnings like:
# ==> default: dpkg-preconfigure: unable to re-open stdin: No such file or directory
# http://serverfault.com/questions/500764/dpkg-reconfigure-unable-to-re-open-stdin-no-file-or-directory
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
locale-gen en_US.UTF-8
dpkg-reconfigure locales

# fix apt warnings like:
# ==> default: dpkg-preconfigure: unable to re-open stdin: No such file or directory
# http://serverfault.com/questions/500764/dpkg-reconfigure-unable-to-re-open-stdin-no-file-or-directory
export LANGUAGE=en_US.UTF-8
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
locale-gen en_US.UTF-8
dpkg-reconfigure locales

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -


sudo apt-get update -qq
sudo apt-get upgrade -y
sudo apt-get install -y python-catkin-tools libsdl2-dev ros-$ROS_DIST-ackermann-msgs ros-$ROS_DIST-serial ros-$ROS_DIST-desktop-full gazebo9 libgazebo9-dev ros-$ROS_DIST-gazebo-ros-control
sudo apt-get install -y libignition-math2-dev

sudo rosdep init

echo "source /opt/ros/$ROS_DIST/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python-visual python-rosinstall python-rosinstall-generator python-wstool build-essential

sudo python -m pip uninstall -y pip
sudo apt-get install -y python-pip
sudo apt-get install -y libsdl2-dev clang-format python-pyqtgraph
sudo python2 -m pip install --upgrade pip --force
sudo python2 -m pip install --no-cache-dir torch autopep8 cython circle-fit

git clone http://github.com/kctess5/range_libc
cd range_libc/pywrapper
./compile.sh
