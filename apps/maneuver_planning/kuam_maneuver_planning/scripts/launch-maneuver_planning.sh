#!/bin/bash
camera="false"
rviz="false"
discription="false"

for cmd in "$@"
do
	if [ "$cmd" == "-h" ]; then
		discription="true"
		break
	elif [ "$cmd" == "camera" ]; then
		camera="true"
	elif [ "$cmd" == "rviz" ]; then
		rviz="true"
	else
		echo "Try 'launch-maneuver_planning.sh -h' for more information."
		exit 0
	fi
done

if [ $discription == "true" ]; then
	echo -e "Usage: launch-maneuver_planning.sh [OPTION] ...\n"
    echo -e "  -h \t\t\t\t Show this help message and exit.\n"

    echo -e "  camera \t\t\t Find landing point by camera (default: false)."

    echo -e "  rviz \t\t\t\t Visualization by using RViz (default: false).\n"

    exit 0
fi

roslaunch kuam_maneuver_planning maneuver_planning.launch using_camera:="$camera" rviz:="$rviz"