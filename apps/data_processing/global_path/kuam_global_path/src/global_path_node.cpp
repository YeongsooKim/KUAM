#include "ros/ros.h"
#include "kuam_global_path/global_path.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "global_path");
    kuam::GlobalPath global_path;

    ros::spin();

    return 0;
} 