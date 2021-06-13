#include "ros/ros.h"
#include "kuam_payload_cmd/payload_cmd.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "payload_cmd_node");
    kuam::Playload playload;

    ros::spin();

    return 0;
}