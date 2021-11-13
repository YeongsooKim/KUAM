cmake_minimum_required(VERSION 3.0.2) 
project(kuam_vehicle_position) 
 
## Compile as C++11, supported in ROS Kinetic and newer 
add_compile_options(-std=c++11) 
 
## Find catkin macros and libraries 
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) 
## is used, also find other catkin packages 
# find_package(Eigen3 REQUIRED) 
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  std_msgs
  nav_msgs
  darknet_ros_msgs
  geometry_msgs
  kuam_msgs
  tf2
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
  nav_msgs
  darknet_ros_msgs
  geometry_msgs
  kuam_msgs
  tf2
  tf2_geometry_msgs
) 
 
########### 
## Build ## 
########### 
 
## Specify additional locations of header files 
## Your package locations should be listed before other locations 
include_directories( 
  include
  ${catkin_INCLUDE_DIRS} 
) 
 
## vehicle_position_node
add_executable(vehicle_position_node src/vehicle_position_node.cpp src/vehicle_position.cpp)
# add_dependencies(kuam_vehicle_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_position_node
 ${catkin_LIBRARIES}
)

## vehicle_position_visual_node
add_executable(vehicle_position_visual_node src/hmi/vehicle_position_visual.cpp)
add_dependencies(vehicle_position_visual_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_position_visual_node ${catkin_LIBRARIES})

## vehicle_position_tf_node
add_executable(vehicle_position_tf_node src/hmi/vehicle_position_tf.cpp)
add_dependencies(vehicle_position_tf_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(vehicle_position_tf_node ${catkin_LIBRARIES})