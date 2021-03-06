#!/bin/bash

px4_dir=${HOME}/git_ws/PX4-Autopilot
source $px4_dir/Tools/setup_gazebo.bash $px4_dir $px4_dir/build/px4_sitl_default

export GAZEBO_MODEL_PATH=${HOME}/kuam_ws/src/KUAM/apps/data_processing/aruco_marker/models:${HOME}/kuam_ws/src/KUAM/apps/data_processing/yolo_vehicle_detection/kuam_vehicle_position/rviz/models:${GAZEBO_MODEL_PATH}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$px4_dir/Tools/sitl_gazebo
