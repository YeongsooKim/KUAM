#ifndef __GLOBAL_PATH_H__
#define __GLOBAL_PATH_H__

#include <ros/ros.h>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <map>

#include <kuam_global_path/utils.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <geographic_msgs/GeoPoint.h>

#include <mavros_msgs/HomePosition.h>

using namespace std;

namespace kuam{

class GlobalPath
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;
    Utils m_utils;

public:
    GlobalPath();
    virtual ~GlobalPath();

private:
    // Subscriber
    ros::Subscriber m_image_sub;
    ros::Subscriber m_home_position_sub;

    // Publisher
    ros::Publisher m_image_pub;

    // Timer
    ros::Timer m_image_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    bool m_using_fake_gps_param;
    double m_latitude_param;
    double m_longitude_param;
    double m_altitude_param;

    // Flag
    bool m_is_home_set;

    geographic_msgs::GeoPoint m_home_position;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void ProcessTimerCallback(const ros::TimerEvent& event);
};
}
#endif //  __GLOBAL_PATH_H__