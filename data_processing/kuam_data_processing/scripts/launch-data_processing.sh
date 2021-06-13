#!/bin/bash

export GAZEBO_MODEL_PATH=${HOME}/kuam_ws/src/KUAM/data_processing/aruco_marker/models:${GAZEBO_MODEL_PATH}

roslaunch kuam_data_processing data_processing.launch