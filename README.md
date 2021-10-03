# KUAM

# python3 환경설정
## Error: dynamic module does not define module export function (PyInit__tf2)
### 1. Upgrade geometry2 to python3
```
cd ~/catkin_ws
source devel/setup.bash
wstool init
wstool set -y src/geometry2 --git https://github.com/ros/geometry2 -v melodic-devel
wstool update src/geometry2
wstool up
rosdep install --from-paths src --ignore-src -y -r

catkin build --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

source devel/setup.bash
```
### 2. Convert kuam workspace settings
```
cd ~/kuam_ws
catkin config -a \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin clean
catkin build
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
```

build PyKDL
```
cd /orocos_kinematics_dynamics/python_orocos_kdl
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DPYTHON_VERSION=3 ..
make -j4
```

copy PyKDL to python3.6/site-packages
```
cd /orocos_kinematics_dynamics/python_orocos_kdl/build/devel/lib/python3/dist-packages
sudo cp PyKDL.so /usr/local/lib/python3.6/dist-packages
export PYTHONPATH=/usr/local/lib/python3.6/dist-packages/:$PYTHONPATH
```
