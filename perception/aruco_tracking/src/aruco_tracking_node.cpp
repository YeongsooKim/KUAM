#include "ros/ros.h"
#include "aruco_tracking/aruco_tracking.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "aruco_tracking");
    kuam::ArucoTracking aruco_tracking;

    ros::spin();

    return 0;
} 