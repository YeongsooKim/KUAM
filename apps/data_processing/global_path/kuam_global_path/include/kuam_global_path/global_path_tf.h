#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>

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

#include "kuam_global_path/utils.h"

using namespace std; 

namespace kuam{
class GlobalPathTfBroadcaster{
private:
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;
    Utils m_utils;

public:
    GlobalPathTfBroadcaster();
    ~GlobalPathTfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_aruco_sub;

    // Timer
    ros::Timer m_tf_broadcaster_timer;

    // Param
    float m_process_freq_param;

    // Flag
    bool m_marker_cb;

    vector<geometry_msgs::TransformStamped> m_marker_tf_stampeds;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> m_transforms;

private:
    void InitFlag();
    bool GetParam();
    void InitROS();

    void ProcessTimerCallback(const ros::TimerEvent& event);
    void MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr);
    void AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __TF_BROADCASTER_H__
