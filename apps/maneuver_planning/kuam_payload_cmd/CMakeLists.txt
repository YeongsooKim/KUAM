cmake_minimum_required(VERSION 3.0.2)
project(kuam_payload_cmd)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
# find_package(Eigen3 REQUIRED)
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

find_package(Boost REQUIRED)
find_package(PythonLibs 2.7)
find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  nav_msgs
  kuam_msgs
  uav_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  kuam_state_machine
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects akuam_state_machinelso need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kuam_payload_cmd
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs kuam_msgs uav_msgs nav_msgs 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## payload_cmd_node
add_executable(payload_cmd_node src/payload_cmd_node.cpp src/payload_cmd.cpp src/utils.cpp)
target_link_libraries(payload_cmd_node ${catkin_LIBRARIES} ${GeographicLib_LIBRARIES})