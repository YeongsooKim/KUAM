#include "ros/ros.h"
#include <ros/spinner.h>
#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

#include "kuam_global_path/global_path_tf.h"

using namespace std;
using namespace kuam;

GlobalPathTfBroadcaster::GlobalPathTfBroadcaster() :
    m_p_nh("~"),
    m_process_freq_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
}

GlobalPathTfBroadcaster::~GlobalPathTfBroadcaster()
{}

void GlobalPathTfBroadcaster::InitFlag()
{
    m_marker_cb = false;
}

bool GlobalPathTfBroadcaster::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_p_nh.getParam("process_freq", m_process_freq_param);

    if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[aruco_tf] m_process_freq_param is NAN"); return false; }

    return true;
}

void GlobalPathTfBroadcaster::InitROS()
{
    // Initialize subscriber
    m_aruco_sub = m_nh.subscribe<tf2_msgs::TFMessage>("aruco_tracking/tf_list", 10, boost::bind(&GlobalPathTfBroadcaster::MarkerCallback, this, _1));

    // Initialize timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &GlobalPathTfBroadcaster::ProcessTimerCallback, this);
}

void GlobalPathTfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_marker_cb) { 
        for (auto tf : m_marker_tf_stampeds){
            AddTransform(tf.header.frame_id, tf.child_frame_id, tf.transform, transform_vector);  
        }
        m_marker_cb = false;
    }
	
    m_tf_broadcaster.sendTransform(transform_vector);
}

void GlobalPathTfBroadcaster::MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr)
{
    m_marker_tf_stampeds.clear();
    for (auto transform : marker_ptr->transforms){
        m_marker_cb = true;
        geometry_msgs::TransformStamped tf;
        m_marker_tf_stampeds.push_back(transform);
    }
}

void GlobalPathTfBroadcaster::AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector)
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
	ros::init (argc, argv, "global_path_tf");
    kuam::GlobalPathTfBroadcaster global_path_tf;

    ros::spin();

    return 0;
}