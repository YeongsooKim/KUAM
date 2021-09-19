#include "ros/ros.h"
#include "kuam_aruco_tracking/tf_broadcaster.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "tf_broadcaster");
    kuam::TfBroadcaster tf_broadcaster;

    ros::spin();

    return 0;
} 