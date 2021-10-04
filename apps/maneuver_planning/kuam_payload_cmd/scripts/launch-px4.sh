#!/bin/bash

if [ "$#" -ne 1 ] && [ "$#" -ne 2 ]; then
    echo "Try 'launch-px4.sh -h' for more information."
    exit 0
else
    if [ "$#" -eq 1 ]; then
        if [ "$1" == "-s" ]; then
            echo "Try 'launch-px4.sh -s -h' for more information."
            exit 0
		elif ! [ "$1" == "-h" ] && ! [ "$1" == "-r" ]; then
            echo "Try 'launch-px4.sh -h' for more information."
            exit 0
        fi
    else
        if ! [ "$1" == "-s" ]; then
            echo "Try 'launch-px4.sh -h' for more information."
            exit 0
        elif ! [ "$2" == "airsim" ] && ! [ "$2" == "gazebo" ] && ! [ "$2" == "-h" ]; then
            echo "Try 'launch-px4.sh -s -h' for more information."
            exit 0
        fi
    fi
fi

simulation="false"
airsim="false"
gazebo="false"

if [ "$1" == "-h" ]; then
    echo -e "Usage: launch-px4.sh [OPTION] ...\n"
    echo -e "  -h, show this help message and exit.\n"
    echo "  -s, px4 sitl. Please select simulator airsim or gazebo."
    echo -e "      Example: launch-px4.sh -s [SIMULATOR]\n"
    echo "  -r, real flight with px4. usage: launch-px4.sh -r"
    echo -e "      Example: launch-px4.sh -r\n"

	exit 0
elif [ "$1" == "-r" ]; then
    simulation="false"
elif [ "$2" == "-h" ]; then
    echo -e "Usage: launch-px4.sh -s [SIMULATOR]\n"
    echo "  airsim, px4 sitl with airsim."
    echo -e "          Example: launch-px4.sh -s airsim\n"
    echo "  gazebo, px4 sitl with gazebo."
    echo -e "          Example: launch-px4.sh -s gazebo\n"

	exit 0
elif [ "$2" == "airsim" ]; then
    simulation="true"
    airsim="true"
elif [ "$2" == "gazebo" ]; then
    simulation="true"
    gazebo="true"
fi

export PX4_HOME_LAT=37.544820
export PX4_HOME_LON=127.078922
export PX4_HOME_ALT=43.479396

if [ $simulation == "false" ] || [ $airsim == "true" ]; then
    roslaunch kuam_payload_cmd px4.launch simulation:="$simulation" gazebo:="$gazebo" fcu_url:="/dev/ttyACM0:57600"
elif [ $gazebo == "true" ]; then
    killall gzclient
    killall gzserver

    source ./launch-common.sh

    roslaunch kuam_payload_cmd px4.launch simulation:="$simulation" gazebo:="$gazebo"
fi
