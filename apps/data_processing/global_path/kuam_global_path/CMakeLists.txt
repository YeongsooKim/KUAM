cmake_minimum_required(VERSION 3.0.2)
project(kuam_global_path)

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
  std_msgs
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
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## global_path_node
add_executable(global_path_node src/global_path_node.cpp src/global_path.cpp src/utils.cpp)
add_dependencies(global_path_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_node
 ${catkin_LIBRARIES}
 ${EIGEN3_INCLUDE_DIRS}
)

## global_path_visual_node
#add_executable(global_path_visual_node src/hmi/global_path_visual.cpp src/utils.cpp)
#target_link_libraries(global_path_visual_node ${catkin_LIBRARIES})

## global_path_tf_broadcaster_node
#add_executable(global_path_tf_broadcaster_node src/hmi/global_path_tf.cpp src/utils.cpp)
#target_link_libraries(global_path_tf_broadcaster_node ${catkin_LIBRARIES})
