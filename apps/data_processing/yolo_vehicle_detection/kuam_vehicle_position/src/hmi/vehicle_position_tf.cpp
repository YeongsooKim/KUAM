#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>
#include <math.h>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <kuam_msgs/VehicleState.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std; 

namespace kuam{
class VehiclePositionTfBroadcaster{
private:
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

public:
    VehiclePositionTfBroadcaster();
    ~VehiclePositionTfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_vehicle_state_sub;

    // Timer
    ros::Timer m_tf_broadcaster_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;

    // Flag
    bool m_marker_cb;
    bool m_vehicle_state_cb;

    vector<geometry_msgs::TransformStamped> m_marker_tf_stampeds;
    geometry_msgs::TransformStamped m_vehicle_tf_stamped;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> m_transforms;

private:
    void InitFlag();
    bool GetParam();
    void InitROS();

    void ProcessTimerCallback(const ros::TimerEvent& event);
    void VehicleStateCallback(const kuam_msgs::VehicleState::ConstPtr vehicle_state_ptr);

    void AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector);
};

VehiclePositionTfBroadcaster::VehiclePositionTfBroadcaster() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[vehicle_position_tf] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

VehiclePositionTfBroadcaster::~VehiclePositionTfBroadcaster()
{}

void VehiclePositionTfBroadcaster::InitFlag()
{
    m_vehicle_state_cb = false;
}

bool VehiclePositionTfBroadcaster::GetParam()
{
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }

    return true;
}

void VehiclePositionTfBroadcaster::InitROS()
{
    // Initialize subscriber
    m_vehicle_state_sub = m_nh.subscribe<kuam_msgs::VehicleState>("vehicle_position/vehicle_state", 10, boost::bind(&VehiclePositionTfBroadcaster::VehicleStateCallback, this, _1));

    // Initialize timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &VehiclePositionTfBroadcaster::ProcessTimerCallback, this);
}

void VehiclePositionTfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_vehicle_state_cb) { AddTransform(m_vehicle_tf_stamped.header.frame_id, m_vehicle_tf_stamped.child_frame_id, m_vehicle_tf_stamped.transform, transform_vector); m_vehicle_state_cb = false; }

	
    m_tf_broadcaster.sendTransform(transform_vector);
}

void VehiclePositionTfBroadcaster::VehicleStateCallback(const kuam_msgs::VehicleState::ConstPtr vehicle_state_ptr)
{
    m_vehicle_state_cb = true;

    m_vehicle_tf_stamped.header.stamp = ros::Time::now();
    m_vehicle_tf_stamped.header.frame_id = vehicle_state_ptr->header.frame_id;
    m_vehicle_tf_stamped.child_frame_id = "vehicle_landing_point";

    m_vehicle_tf_stamped.transform.translation.x = vehicle_state_ptr->vehicle.position.x;
    m_vehicle_tf_stamped.transform.translation.y = vehicle_state_ptr->vehicle.position.y;
    m_vehicle_tf_stamped.transform.translation.z = vehicle_state_ptr->vehicle.position.z;
    m_vehicle_tf_stamped.transform.rotation.x = vehicle_state_ptr->vehicle.orientation.x;
    m_vehicle_tf_stamped.transform.rotation.y = vehicle_state_ptr->vehicle.orientation.y;
    m_vehicle_tf_stamped.transform.rotation.z = vehicle_state_ptr->vehicle.orientation.z;
    m_vehicle_tf_stamped.transform.rotation.w = vehicle_state_ptr->vehicle.orientation.w;
}



void VehiclePositionTfBroadcaster::AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector)
{
	geometry_msgs::TransformStamped tf_stamped;

	tf_stamped.header.stamp = ros::Time::now();
	tf_stamped.header.frame_id = frame_id;
	tf_stamped.child_frame_id = child_id;
    tf_stamped.transform = tf;

	vector.push_back(tf_stamped);
}
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "vehicle_position_tf");
    kuam::VehiclePositionTfBroadcaster vehicle_position_tf;

    ros::spin();

    return 0;
} 