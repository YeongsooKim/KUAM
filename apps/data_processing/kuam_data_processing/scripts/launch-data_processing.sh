#!/bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ] && [ "$#" -ne 3 ] && [ "$#" -ne 4 ] && [ "$#" -ne 6] ; then
    echo "Try 'launch-data_processing.sh -h' for more information."
    exit 0
else
    if [ "$#" -eq 1 ]; then
		if ! [ "$1" == "-h" ]; then
            echo "Try 'launch-data_processing.sh -h' for more information."
            exit 0
        fi
    else
        if ! [ "$1" == "--aruco" ]; then
            echo "Try 'launch-data_processing.sh -h' for more information."
            exit 0
        elif ! [ "$2" == "true" ] && ! [ "$2" == "false" ]; then
            echo "Try 'launch-data_processing.sh -h' for more information."
            exit 0
        elif [ "$2" == "true" ]; then
            if [ "$#" -eq 3 ]; then
        		if ! [ "$3" == "-h" ]; then
                    echo "Try 'launch-data_processing.sh --aruco true -h' for more information."
                    exit 0
                fi
            else
                if ! [ "$3" == "-t" ]; then
                    echo "Try 'launch-data_processing.sh --aruco true -h' for more information."
                    exit 0
                elif ! [ "$4" == "gazebo" ] && ! [ "$4" == "logging" ] && ! [ "$4" == "device" ]; then
                    echo "Try 'launch-data_processing.sh --aruco true -h' for more information."
                    exit 0
                fi
            fi
        fi
    fi
fi

using="false"
gazebo="true"
logging="false"
device="false"

if [ "$1" == "-h" ]; then
    echo -e "Usage: launch-data_processing.sh [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  --aruco, find landing point by using ArUco marker or not (using only GPS). Please select true or false."
    echo -e "      Example: launch-data_processing.sh --aruco true\n"

	exit 0
elif [ "$2" == "false" ]; then
    using="false"
elif [ "$3" == "-h" ]; then
    echo -e "Usage: launch-data_processing.sh --aruco true [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  -t, What data type is used for the camera. Please select gazebo or logging or device."
    echo -e "      Example: launch-data_processing.sh --aruco true -t gazebo"

	exit 0
elif [ "$4" == "gazebo" ]; then
    using="true"
    gazebo="true"
    logging="false"
    device="false"
elif [ "$4" == "logging" ]; then
    using="true"
    gazebo="false"
    logging="true"
    device="false"
elif [ "$4" == "device" ]; then
    using="true"
    gazebo="false"
    logging="false"
    device="true"
fi

rviz="false"
if [ "$5" == "--rviz" ]; then
	if [ "$6" == "true" ]; then
		rviz="true"
	fi
fi

if [ $using == "false" ]; then
    roslaunch kuam_data_processing data_processing.launch using_aruco:="$using"
else
    roslaunch kuam_data_processing data_processing.launch using_aruco:="$using" using_gazebo_data:="$gazebo" using_logging_data:="$logging" using_device_data:="$device" rviz:="$rviz"
fi
