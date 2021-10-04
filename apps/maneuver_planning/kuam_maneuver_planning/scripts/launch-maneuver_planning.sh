#!/bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ] && [ "$#" -ne 3 ] && [ "$#" -ne 4 ]; then
    echo "Try 'launch-maneuver_planning.sh -h' for more information."
    exit 0
else
    if [ "$#" -eq 1 ]; then
		if ! [ "$1" == "-h" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        fi
    else
        if ! [ "$1" == "--aruco" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        elif ! [ "$2" == "true" ] && ! [ "$2" == "false" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        elif [ "$2" == "true" ]; then
            if [ "$#" -eq 3 ]; then
        		if ! [ "$3" == "-h" ]; then
                    echo "Try 'launch-maneuver_planning.sh --aruco true -h' for more information."
                    exit 0
                fi
            else
                if ! [ "$3" == "-t" ]; then
                    echo "Try 'launch-maneuver_planning.sh --aruco true -h' for more information."
                    exit 0
                elif ! [ "$4" == "gazebo" ] && ! [ "$4" == "real" ] && ! [ "$4" == "exp" ]; then
                    echo "Try 'launch-maneuver_planning.sh --aruco true -h' for more information."
                    exit 0
                fi
            fi
        fi
    fi
fi

using="false"
gazebo="true"
real="false"
exp="false"

if [ "$1" == "-h" ]; then
    echo -e "Usage: launch-maneuver_planning.sh [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  --aruco, find landing point by using ArUco marker or not (using only GPS). Please select true or false."
    echo -e "      Example: launch-maneuver_planning.sh --aruco true\n"

	exit 0
elif [ "$2" == "false" ]; then
    using="False"
elif [ "$3" == "-h" ]; then
    echo -e "Usage: launch-maneuver_planning.sh --aruco true [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  -t, What frame is parent frame to camera_link; parent_frame to camera_link translation. Please select gazebo or real or exp."
    echo -e "      Example: launch-maneuver_planning.sh --aruco true -t gazebo"

    exit 0
elif [ "$4" == "gazebo" ]; then
    using="True"
    gazebo="true"
    real="false"
    exp="false"
elif [ "$4" == "real" ]; then
    using="True"
    gazebo="false"
    real="true"
    exp="false"
elif [ "$4" == "exp" ]; then
    using="True"
    gazebo="false"
    real="false"
    exp="true"
fi

if [ $using == "false" ]; then
    roslaunch kuam_maneuver_planning maneuver_planning.launch using_camera:="$using"
else
    roslaunch kuam_maneuver_planning maneuver_planning.launch using_camera:="$using" is_gazebo:="$gazebo" is_real:="$real" is_exp:="$exp"
fi