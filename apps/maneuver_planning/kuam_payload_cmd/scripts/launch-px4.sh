#!/bin/bash
sim="false"
discription="false"

for cmd in "$@"
do
	if [ "$cmd" == "-h" ]; then
		discription="true"
		break
	elif [ "$cmd" == "sim" ]; then
		sim="true"
	else
		echo "Try 'launch-px4.sh -h' for more information."
		exit 0
	fi
done

if [ "$discription" == "true" ]; then
	echo -e "Usage: launch-maneuver_planning.sh [OPTION] ...\n"
    echo -e "  -h \t\t\t\t Show this help message and exit.\n"

    echo -e "  sim \t\t\t\t Working in simulation (default: false).\n"

    exit 0
fi

home_lat=37.544820
home_lon=127.078922
home_alt=43.479396

export PX4_HOME_LAT=$home_lat
export PX4_HOME_LON=$home_lon
export PX4_HOME_ALT=$home_alt

if [ $sim == "false"]; then
    roslaunch kuam_payload_cmd px4.launch simulation:="$sim" fcu_url:="/dev/ttyACM0:57600"
else
    killall gzclient
    killall gzserver

    source ./launch-common.sh

    roslaunch kuam_payload_cmd px4.launch simulation:="$sim" home_lon:=$home_lon home_lat:=$home_lat home_alt:=$home_alt
fi