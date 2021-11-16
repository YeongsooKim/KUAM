# Ubuntu Server Install
## PreProcessing
### 1. Ubuntu server image download
[ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img](https://drive.google.com/file/d/1dEHVsipzj39RlXR2r8A3cujXgGlD0aHF/view?usp=sharing)

### 2. SD Card formatter
[sdcard formatter](https://www.sdcard.org/downloads/formatter/)

Download program and run the program

### 3. Raseberry Pi Imager를 사용하여 굽기
[https://ubuntu.com/download/raspberry-pi](https://ubuntu.com/download/raspberry-pi)

![raspberry_pi_imager_download](https://github.com/YeongsooKim/KUAM/blob/YeongsooKim/img/raspberry_pi_imager_download.png)

Implement Raspberry Pi Imager and choose operating system (download above link 'ubuntu-18.04.5-preinstalled-server-arm64+raspi4.img').
Finally select SD Card and push the 'WRITE' button

![raspberry_pi_imager](https://github.com/YeongsooKim/KUAM/blob/YeongsooKim/img/raspberry_pi_imager.png)

### 4. Command Network Setting
netplan 수정
```
cd /etc/netplan
sudo cp 50-cloud-init.yaml 50-cloud-init.yaml.bak
sudo vim 50-cloud-init.yamlcd /etc/netplan

******************** 50-cloud-init.yaml ********************
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            dhcp4: true
            access-points:
                "pav":
                    password: "abcd1234"
******************** 50-cloud-init.yaml ********************

sudo netplan generate
sudo netplan apply
sudo systemctl daemon-reload
```

![command_network](https://github.com/YeongsooKim/KUAM/blob/YeongsooKim/img/command_network.png)


### 5. Password
```
sudo passwd root
sudo passwd $USER
```


# python3 환경설정
## Missing package install for python3
### 1. apt update & install python3 package 
```
sudo apt install python3-pip python3-all-dev python3-rospkg
sudo apt update
sudo apt upgrade
sudo apt install ros-melodic-desktop-full --fix-missing
```

## Error: dynamic module does not define module export function (PyInit__tf2)
### 1. Upgrade geometry2 to python3

arm 구조가 아닐때:
```
cd ~/catkin_ws
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel
wstool update src/geometry2
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin clean
catkin config --extend /opt/ros/melodic
catkin config -a \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
source devel/setup.bash
```

arm 구조일 때:
```
cd ~/catkin_ws
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel
wstool update src/geometry2
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin clean
catkin config --extend /opt/ros/melodic
catkin config -a \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
source devel/setup.bash
```

### 2. Convert kuam workspace settings
arm 구조가 아닐때:
```
# CMAKE_PREFIX_PATH 가 자동으로 잡히도록 reboot (부정확함)
reboot

cd ~/kuam_ws
catkin clean
catkin config -a \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin build
source devel/setup.bash
```

arm 구조일 때:

```
# CMAKE_PREFIX_PATH 가 자동으로 잡히도록 reboot (부정확함)
reboot

cd ~/kuam_ws
catkin clean
catkin config -a \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.6m.so
catkin build
source devel/setup.bash
```

### 3. PyKDL update
remove previous PyKDL
`sudo apt remove ros-melodic-python-orocos-kdl`

git clone python3 PyKDL
`git clone https://github.com/orocos/orocos_kinematics_dynamics.git`

git submodule init && git submodule update in pybind11 folder
```
cd orocos_kinematics_dynamics/python_orocos_kdl/pybind11
git submodule init && git submodule update
```

build & install orocos_kdl
```
cd orocos_kdl
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install

echo "LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc
```
Explanation for adding /usr/local/lib to LD_LIBRARY_PATH: [link](https://stackoverflow.com/questions/1099981/why-cant-python-find-shared-objects-that-are-in-directories-in-sys-path)

build PyKDL
```
cd /orocos_kinematics_dynamics/python_orocos_kdl
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3 ..
make -j4
sudo make install

sudo cp /usr/local/lib/python3/dist-packages/PyKDL.so /usr/local/lib/python3.6/dist-packages/PyKDL.so
echo "export PYTHONPATH=/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH" >> ~/.bashrc
```


# Dependency
## No module named 'scipy'
`sudo apt-get install python3-scipy`

# Path
## File "/opt/ros/melodic/lib/python2.7/dist-packages/smach_ros/__init__.py", line 52, ModuleNotFoundError: No module named 'util'
convert to relative path

from 
 ```
 ### Core classes
 from util import set_preempt_handler
 
 ### Top-level Containers / Wrappers
 from action_server_wrapper import ActionServerWrapper
 from introspection import IntrospectionClient, IntrospectionServer
 
 ### State Classes
 from simple_action_state import SimpleActionState
 from service_state import ServiceState
 from monitor_state import MonitorState
 from condition_state import ConditionState
```

to
```
 ### Core classes
 from .util import set_preempt_handler
 
 ### Top-level Containers / Wrappers
 from .action_server_wrapper import ActionServerWrapper
 from .introspection import IntrospectionClient, IntrospectionServer
 
 ### State Classes
 from .simple_action_state import SimpleActionState
 from .service_state import ServiceState
 from .monitor_state import MonitorState
 from .condition_state import ConditionState
```
