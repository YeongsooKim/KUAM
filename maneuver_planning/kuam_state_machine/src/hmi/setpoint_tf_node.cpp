#include "ros/ros.h"
#include "kuam_state_machine/setpoint_tf.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "setpoint_tf");
    kuam::TfBroadcaster setpoint_tf;

    ros::spin();

    return 0;
} 