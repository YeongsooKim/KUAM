#include "ros/ros.h"
#include <ros/spinner.h>
#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

#include "kuam_aruco_tracking/aruco_tf.h"

using namespace std;
using namespace kuam;

ArucoTfBroadcaster::ArucoTfBroadcaster() :
    m_p_nh("~"),
    m_process_freq_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_tf] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

ArucoTfBroadcaster::~ArucoTfBroadcaster()
{}

void ArucoTfBroadcaster::InitFlag()
{
    m_marker_cb = false;
}

bool ArucoTfBroadcaster::GetParam()
{
    string nd_name = ros::this_node::getName();

    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }

    return true;
}

void ArucoTfBroadcaster::InitROS()
{
    // Initialize subscriber
    m_aruco_sub = m_nh.subscribe<tf2_msgs::TFMessage>("aruco_tracking/tf_list", 10, boost::bind(&ArucoTfBroadcaster::MarkerCallback, this, _1));

    // Initialize timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &ArucoTfBroadcaster::ProcessTimerCallback, this);
}

void ArucoTfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
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

void ArucoTfBroadcaster::MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr)
{
    m_marker_tf_stampeds.clear();
    for (auto transform : marker_ptr->transforms){
        m_marker_cb = true;
        geometry_msgs::TransformStamped tf;
        m_marker_tf_stampeds.push_back(transform);
    }
}

void ArucoTfBroadcaster::AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector)
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
	ros::init (argc, argv, "aruco_tf");
    kuam::ArucoTfBroadcaster aruco_tf;

    ros::spin();

    return 0;
} 