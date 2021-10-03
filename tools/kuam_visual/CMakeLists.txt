cmake_minimum_required(VERSION 3.0.2)
project(kuam_visual)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  roscpp
  sensor_msgs
  kuam_msgs
  mavros_msgs
  nav_msgs
  kuam_aruco_tracking
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
  LIBRARIES kuam_visual
  CATKIN_DEPENDS 
  geometry_msgs 
  geographic_msgs
  roscpp 
  sensor_msgs 
  kuam_msgs
  mavros_msgs
  nav_msgs
  kuam_aruco_tracking
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

## kuam_visual_node
# add_executable(kuam_visual_node src/kuam_visual.cpp src/utils.cpp)
# target_link_libraries(kuam_visual_node ${catkin_LIBRARIES})

## aruco_visual_node
# add_executable(aruco_visual_node src/aruco_visual.cpp src/utils.cpp)
# target_link_libraries(aruco_visual_node ${catkin_LIBRARIES})
