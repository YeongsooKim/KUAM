#ifndef __ARUCO_TF_H__
#define __ARUCO_TF_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <kuam_msgs/ArucoStates.h>

using namespace std; 

namespace kuam{
class ArucoTfBroadcaster{
private:
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

public:
    ArucoTfBroadcaster();
    ~ArucoTfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_aruco_tf_sub;
    ros::Subscriber m_aruco_states_sub;

    // Timer
    ros::Timer m_tf_broadcaster_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;

    // Flag
    bool m_marker_cb;
    bool m_setpoint_cb;

    vector<geometry_msgs::TransformStamped> m_marker_tf_stampeds;
    geometry_msgs::TransformStamped m_setpoint_tf_stamped;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> m_transforms;

private:
    void InitFlag();
    bool GetParam();
    void InitROS();

    void ProcessTimerCallback(const ros::TimerEvent& event);
    void MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr);
    void ArucoStatesCallback(const kuam_msgs::ArucoStates::ConstPtr &aruco_msg_ptr);

    void AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __ARUCO_TF_H__