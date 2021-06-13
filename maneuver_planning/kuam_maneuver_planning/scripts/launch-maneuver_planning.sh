#!/bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ]; then
    echo "Try 'launch-maneuver_planning.sh -h' for more information."
    exit 0
else
    if [ "$#" -eq 1 ]; then
        if [ "$1" == "--aruco" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
		elif ! [ "$1" == "-h" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        fi
    else
        if ! [ "$1" == "--aruco" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        elif ! [ "$2" == "true" ] && ! [ "$2" == "false" ] && ! [ "$2" == "-h" ]; then
            echo "Try 'launch-maneuver_planning.sh -h' for more information."
            exit 0
        fi
    fi
fi

using="false"

if [ "$1" == "-h" ] || [ "$2" == "-h" ]; then
    echo -e "Usage: launch-maneuver_planning.sh [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  --aruco, find landing point by using ArUco marker or not (using only GPS). Please select true or false."
    echo "      Example1: launch-maneuver_planning.sh --aruco true"
    echo -e "      Example2: launch-maneuver_planning.sh --aruco false\n"

	exit 0
elif [ "$2" == "true" ]; then
    using="true"
elif [ "$2" == "false" ]; then
    using="false"
fi

roslaunch kuam_maneuver_planning maneuver_planning.launch using_aruco:="$using"