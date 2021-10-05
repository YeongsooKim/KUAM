#include "ros/ros.h"
#include "kuam_vehicle_position/vehicle_position.h"
#include <ros/spinner.h>

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "vehicle_position");
    kuam::VehiclePosition vehicle_pos;

    ros::spin();

    return 0;
} 