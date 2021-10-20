cmake_minimum_required(VERSION 3.0.2)
project(kuam_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  geographic_msgs
  message_generation
  sensor_msgs
  std_msgs
  jsk_recognition_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ArucoState.msg
  ArucoStates.msg
  ArucoVisual.msg
  ArucoVisuals.msg
  Completion.msg
  FittingPlane.msg
  FittingPlanes.msg
  LandingState.msg
  marker_ids.msg
  Qgc.msg
  QgcDatum.msg
  Setpoint.msg
  Status.msg
  Task.msg
  TaskList.msg
  VehicleState.msg
  Waypoint.msg
  Waypoints.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  EmergyRequest.srv
  GlobalPathSync.srv
  LandRequest.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  geographic_msgs
  jsk_recognition_msgs
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
  CATKIN_DEPENDS 
  message_runtime
  std_msgs
  geometry_msgs
  geographic_msgs
  sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

