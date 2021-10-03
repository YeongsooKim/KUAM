#include "ros/ros.h"
#include <ros/spinner.h>
#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

#include "kuam_state_machine/state_machine_tf.h"

using namespace std;
using namespace kuam;

SetpointTfBroadcaster::SetpointTfBroadcaster() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[state_machine_tf] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

SetpointTfBroadcaster::~SetpointTfBroadcaster()
{}

void SetpointTfBroadcaster::InitFlag()
{
    m_setpoint_cb = false;
}

bool SetpointTfBroadcaster::GetParam()
{
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }

    return true;
}

void SetpointTfBroadcaster::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    string ns_name = ros::this_node::getNamespace();
    
    // Initialize subscriber
    // m_setpoint_sub = 
    //     m_nh.subscribe<kuam_msgs::Setpoint>("state_machine/setpoint", 10, boost::bind(&SetpointTfBroadcaster::SetpointCallback, this, _1));
    
    // Initialize timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &SetpointTfBroadcaster::ProcessTimerCallback, this);
}

void SetpointTfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_setpoint_cb) { AddTransform(m_setpoint_tf_stamped.header.frame_id, m_setpoint_tf_stamped.child_frame_id, m_setpoint_tf_stamped.transform, transform_vector); m_setpoint_cb = false; }
	
    m_tf_broadcaster.sendTransform(transform_vector);
}

void SetpointTfBroadcaster::AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector)
{
	geometry_msgs::TransformStamped tf_stamped;

	tf_stamped.header.stamp = ros::Time::now();
	tf_stamped.header.frame_id = frame_id;
	tf_stamped.child_frame_id = child_id;
    tf_stamped.transform = tf;

	vector.push_back(tf_stamped);
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "state_machine_tf");
    kuam::SetpointTfBroadcaster state_machine_tf;

    ros::spin();

    return 0;
} 