cmake_minimum_required(VERSION 3.0.2)
project(kuam_state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  uav_msgs
  kuam_msgs
  nav_msgs
  sensor_msgs
  tf2 
  tf2_ros
  mavros_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  uav_msgs 
  kuam_msgs 
  nav_msgs
  sensor_msgs
  tf2 
  tf2_ros
  mavros_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## mission_manager_node

## state_machine_node
catkin_install_python(PROGRAMS 
  src/state_machine.py
  src/state/flight.py
  src/state/hovering.py
  src/state/landing.py
  src/state/standby.py
  src/state/state.py
  src/state/takeoff.py
  src/utils/util_geometry.py
  src/utils/util_state.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## state_machine_tf_node
add_executable(state_machine_tf_node src/hmi/state_machine_tf.cpp)
target_link_libraries(state_machine_tf_node ${catkin_LIBRARIES})

## state_machine_visual
add_executable(state_machine_visual_node src/hmi/state_machine_visual.cpp src/utils/util_visual.cpp)
target_link_libraries(state_machine_visual_node ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
