#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>

#include <mavros_msgs/HomePosition.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <kuam_msgs/Setpoint.h>
#include "kuam_state_machine/utils.h"

using namespace std; 

namespace kuam{
class TfBroadcaster{
private:
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;
    Utils m_utils;

public:
    TfBroadcaster();
    ~TfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_setpoint_sub;

    // Timer
    ros::Timer m_tf_broadcaster_timer;

    // Param
    float m_process_freq_param;

    // Flag
    bool m_setpoint_cb;

    geometry_msgs::TransformStamped m_setpoint_tf_stamped;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> m_transforms;

private:
    void InitFlag();
    bool GetParam();
    void InitROS();

    void ProcessTimerCallback(const ros::TimerEvent& event);

    void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr);
    
    void AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __TF_BROADCASTER_H__
