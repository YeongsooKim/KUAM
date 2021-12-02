#! /usr/bin/env bash

# Coral. Get started with the USB Accelerator. https://coral.ai/docs/accelerator/get-started/#requirements

## Install the Edge TPU runtime

### 1. Add our Debian package repository to your system:
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get --quiet -y update

### 2. Install the Edge TPU runtime:
sudo apt-get --quiet -y install libedgetpu1-std


## Install the PyCoral library
sudo apt-get --quiet -y install python3-pycoral


# dependencies
sudo apt-get --quiet -y update
sudo apt-get -y --quiet --no-install-recommends install \
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

## update pip3
pip3 install --upgrade pip

# script directory
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )

# Python3 dependencies
python3 -m pip install --user -r ${DIR}/requirements_pip3.txt
