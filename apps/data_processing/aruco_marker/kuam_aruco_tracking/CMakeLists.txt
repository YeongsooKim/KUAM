cmake_minimum_required(VERSION 3.0.2)
project(kuam_aruco_tracking)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  image_transport
  cv_bridge
  std_msgs
  sensor_msgs
  kuam_msgs
  tf2 
  tf2_ros
)
# message("OpenCV version: ${OpenCV_VERSION}")

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
  geometry_msgs
  roscpp
  kuam_msgs
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  tf2 
  tf2_ros
  DEPENDS
    OpenMP
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
  ${OpenCV_INCLUDE_DIRS}
)

## aruco_tracking_node
add_executable(aruco_tracking_node 
  src/aruco_tracking_node.cpp 
  src/aruco_tracking.cpp 
  src/parser.cpp 
  src/target.cpp
  src/frequency_degree.cpp
  src/utils/util_marker.cpp
  src/utils/util_setpoint.cpp
)

add_dependencies(aruco_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_tracking_node
 ${catkin_LIBRARIES}
 ${EIGEN3_INCLUDE_DIRS}
 ${OpenCV_LIBRARIES}
 ${OpenMP_LIBS}
)

## aruco_visual_node
add_executable(aruco_visual_node src/hmi/aruco_visual.cpp src/utils/util_geometry.cpp)
target_link_libraries(aruco_visual_node ${catkin_LIBRARIES})

## aruco_tf_node
add_executable(aruco_tf_node src/hmi/aruco_tf.cpp)
target_link_libraries(aruco_tf_node ${catkin_LIBRARIES})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)