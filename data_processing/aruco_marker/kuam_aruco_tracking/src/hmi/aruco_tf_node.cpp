#include "ros/ros.h"
#include "kuam_aruco_tracking/aruco_tf.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "aruco_tf");
    kuam::TfBroadcaster aruco_tf;

    ros::spin();

    return 0;
} 