#! /usr/bin/env bash


# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)


sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get --quiet -y update && sudo apt-get --quiet -y install python3-catkin-tools

sudo DEBIAN_FRONTEND=nointeractive apt-get -y --quiet --no-install-recommends install \
	astyle \
	build-essential \
	ccache \
	cmake \
	cppcheck \
	file \
	g++ \
	gcc \
	gdb \
	git \
	make \
	python-pip \
	python3 \
	python3-dev \
	python3-pip \
	python3-setuptools \
	python3-wheel \
	python3-all-dev \
	python3-rospkg \
	rsync \
	shellcheck \
	unzip \
	zip \
	curl \
	;


# install ubuntu melodic
## sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt-get --quiet -y update
sudo apt-get -y --quiet install ros-melodic-desktop-full

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get -y --quiet update

sudo apt-get -y --quiet install \
	python-rosdep \
	python-rosinstall \
	python-rosinstall-generator \
	python-wstool \
	;

sudo rosdep init
rosdep update


# kuam_ws dependency
sudo apt-get -y --quiet install \
	ros-melodic-jsk-recognition-msgs \
	ros-melodic-jsk-rviz-plugins \
	ros-melodic-geographic-msgs \
	ros-melodic-tf2-sensor-msgs \
	;


python -m pip install -r ${DIR}/requirements.txt

pip3 install --user future
pip3 install --user empy
sudo apt-get -y --quiet install \
	python3-scipy
