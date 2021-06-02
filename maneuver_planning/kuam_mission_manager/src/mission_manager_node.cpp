#include "ros/ros.h"
#include "kuam_mission_manager/mission_manager.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "manuever_planning");
    kuam::Maneuver manuever_planning;

    ros::spin();

    return 0;
} 