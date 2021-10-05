cmake_minimum_required(VERSION 3.0.2) 
project(pub_img_stream) 
 
## Compile as C++11, supported in ROS Kinetic and newer 
add_compile_options(-std=c++11) 
 
## Find catkin macros and libraries 
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz) 
## is used, also find other catkin packages 
# find_package(Eigen3 REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  image_transport
  cv_bridge
  std_msgs 
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
  roscpp  
  image_transport
  cv_bridge
  std_msgs 
) 
 
########### 
## Build ## 
########### 
 
## Specify additional locations of header files 
## Your package locations should be listed before other locations 
include_directories( 
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}
) 
 
## aruco_tracking_node
add_executable(pub_img_stream src/pub_img_stream.cpp)
# add_dependencies(pub_img_stream ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_img_stream
 ${catkin_LIBRARIES}
 ${OpenCV_LIBRARIES}
)