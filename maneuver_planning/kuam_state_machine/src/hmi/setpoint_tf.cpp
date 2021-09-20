#include "kuam_state_machine/setpoint_tf.h"
#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace kuam;

TfBroadcaster::TfBroadcaster() :
    m_p_nh("~"),
    m_process_freq_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
}

TfBroadcaster::~TfBroadcaster()
{}

void TfBroadcaster::InitFlag()
{
    m_setpoint_cb = false;
}

bool TfBroadcaster::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_p_nh.getParam("process_freq", m_process_freq_param);

    if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[setpoint_tf] m_process_freq_param is NAN"); return false; }

    return true;
}

void TfBroadcaster::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    string ns_name = ros::this_node::getNamespace();
    
    // Initialize subscriber
    m_setpoint_sub = 
        m_nh.subscribe<kuam_msgs::Setpoint>("state_machine/setpoint", 10, boost::bind(&TfBroadcaster::SetpointCallback, this, _1));
    
    // Initialize timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &TfBroadcaster::ProcessTimerCallback, this);
}

void TfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_setpoint_cb) { AddTransform(m_setpoint_tf_stamped.header.frame_id, m_setpoint_tf_stamped.child_frame_id, m_setpoint_tf_stamped.transform, transform_vector); m_setpoint_cb = false; }
	
    m_tf_broadcaster.sendTransform(transform_vector);
}

void TfBroadcaster::SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr)
{
    if (setpoint_ptr->is_global){
        return;
    }

    m_setpoint_cb = true;

    m_setpoint_tf_stamped.header.stamp = ros::Time::now();
    m_setpoint_tf_stamped.header.frame_id = setpoint_ptr->header.frame_id;
    m_setpoint_tf_stamped.child_frame_id = "landing_point";

    m_setpoint_tf_stamped.transform.translation.x = setpoint_ptr->pose.position.x;
    m_setpoint_tf_stamped.transform.translation.y = setpoint_ptr->pose.position.y;
    m_setpoint_tf_stamped.transform.translation.z = setpoint_ptr->pose.position.z;
    m_setpoint_tf_stamped.transform.rotation.x = setpoint_ptr->pose.orientation.x;
    m_setpoint_tf_stamped.transform.rotation.y = setpoint_ptr->pose.orientation.y;
    m_setpoint_tf_stamped.transform.rotation.z = setpoint_ptr->pose.orientation.z;
    m_setpoint_tf_stamped.transform.rotation.w = setpoint_ptr->pose.orientation.w;
}

void TfBroadcaster::AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector)
{
	geometry_msgs::TransformStamped tf_stamped;

	tf_stamped.header.stamp = ros::Time::now();
	tf_stamped.header.frame_id = frame_id;
	tf_stamped.child_frame_id = child_id;
    tf_stamped.transform = tf;

	vector.push_back(tf_stamped);
}
