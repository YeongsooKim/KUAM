cmake_minimum_required(VERSION 3.0.2)
project(kuam_state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(Eigen3 REQUIRED)
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
  tf2_sensor_msgs
  tf2_geometry_msgs
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
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## mission_manager_node

## setpoint_tf_node
add_executable(setpoint_tf_node src/hmi/setpoint_tf.cpp src/hmi/setpoint_tf_node.cpp src/utils.cpp)
target_link_libraries(setpoint_tf_node ${catkin_LIBRARIES})

## setpoint_visual
add_executable(setpoint_visual_node src/hmi/setpoint_visual.cpp src/utils.cpp)
target_link_libraries(setpoint_visual_node ${catkin_LIBRARIES})
