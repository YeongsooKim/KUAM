#!/bin/bash
aruco="false"
yolo="false"
gps="true"
sim="false"
rviz="false"
discription="false"

for cmd in "$@"
do
	if [ "$cmd" == "-h" ]; then
		discription="true"
		break
	elif [ "$cmd" == "aruco" ]; then
		aruco="true"
        gps="false"
	elif [ "$cmd" == "yolo" ]; then
		yolo="true"
        gps="false"
	elif [ "$cmd" == "sim" ]; then
		sim="true"
	elif [ "$cmd" == "rviz" ]; then
		rviz="true"
	else
		echo "Try 'launch-data_processing.sh -h' for more information."
		exit 0
	fi
done

if [ $discription == "true" ]; then
	echo -e "Usage: launch-data_processing.sh [OPTION] ...\n"
    echo -e "  -h \t\t\t\t Show this help message and exit.\n"

    echo -e "  aruco \t\t\t Find landing point by using ArUco marker (default: false)."
    echo -e "  yolo \t\t\t\t Find landing point by using YOLO (default: false)."
    echo -e "  gps \t\t\t\t Find landing point by using GPS only (default: true).\n"

    echo -e "  sim \t\t\t\t Working in simulation (default: false).\n"

    echo -e "  rviz \t\t\t\t Visualization by using RViz (default: false).\n"

    exit 0
fi

roslaunch kuam_data_processing data_processing.launch using_aruco:="$aruco" using_yolo:="$yolo" gps_only:="$gps" simulation:="$sim" rviz:="$rviz"
