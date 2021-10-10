#include <Eigen/Dense>
#include <cmath>

// Messages
#include <geometry_msgs/Pose.h>

#include "kuam_global_tf/global_tf.h"

using namespace std;
using namespace kuam;

TfBroadcaster::TfBroadcaster() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[global_tf] Fail GetParam %s", m_err_param.c_str());
    InitROS();
    InitStaticTf();
}

TfBroadcaster::~TfBroadcaster()
{}

bool TfBroadcaster::InitFlag()
{
    m_is_home_set = false;
    m_base_cb = false;

    return true;
}

bool TfBroadcaster::GetParam()
{
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("is_real", m_is_real_param)) { m_err_param = "is_real"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_x_m", m_extrinsic_imu_to_camera_x_m_param)) { m_err_param = "extrinsic_imu_to_camera_x_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_y_m", m_extrinsic_imu_to_camera_y_m_param)) { m_err_param = "extrinsic_imu_to_camera_y_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_z_m", m_extrinsic_imu_to_camera_z_m_param)) { m_err_param = "extrinsic_imu_to_camera_z_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_r_deg", m_extrinsic_imu_to_camera_r_deg_param)) { m_err_param = "extrinsic_imu_to_camera_r_deg"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_p_deg", m_extrinsic_imu_to_camera_p_deg_param)) { m_err_param = "extrinsic_imu_to_camera_p_deg"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_y_deg", m_extrinsic_imu_to_camera_y_deg_param)) { m_err_param = "extrinsic_imu_to_camera_y_deg"; return false; }
    if (!m_nh.getParam("/sim", m_is_sim_param)) { m_err_param = "sim"; return false; }
    if (m_is_sim_param){
        m_is_home_set = true;
        if (!m_nh.getParam("/home_lon", m_home_position.longitude)) { m_err_param = "home_lon"; return false; }
        if (!m_nh.getParam("/home_lat", m_home_position.latitude)) { m_err_param = "home_lat"; return false; }
        if (!m_nh.getParam("/home_alt", m_home_position.altitude)) { m_err_param = "home_alt"; return false; }
        ROS_WARN_STREAM("[tf_broadcaster] Home set");
    }

    return true;
}

bool TfBroadcaster::InitROS()
{
    // Initialize subscriber
    if (!m_is_sim_param){
        m_home_position_sub = 
            m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&TfBroadcaster::HomePositionCallback, this, _1));
        ros::Rate rate(0.2);
        while (ros::ok() && !m_is_home_set){
            ROS_WARN_STREAM("[tf_broadcaster] Requesting home set");
            ros::spinOnce();
            rate.sleep();
        }
    }

    m_ego_global_pos_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&TfBroadcaster::EgoGlobalCallback, this, _1));
    m_local_pose_sub = m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&TfBroadcaster::EgoLocalCallback, this, _1));
    
    // Init timer
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &TfBroadcaster::ProcessTimerCallback, this);
    return true;
}

bool TfBroadcaster::InitStaticTf()
{
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> transform_vector;

    geometry_msgs::TransformStamped tf_stamped;

    // camera_link static transformation
    if (m_is_real_param){
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = m_extrinsic_imu_to_camera_x_m_param;
        tf_stamped.transform.translation.y = m_extrinsic_imu_to_camera_y_m_param;
        tf_stamped.transform.translation.z = m_extrinsic_imu_to_camera_z_m_param;
        
        tf2::Quaternion q;
        double DEG2RAD = M_PI/180.0;
        q.setRPY(m_extrinsic_imu_to_camera_r_deg_param*DEG2RAD,
                    m_extrinsic_imu_to_camera_p_deg_param*DEG2RAD,
                    m_extrinsic_imu_to_camera_y_deg_param*DEG2RAD);
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
    }
    else {
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = 0.1;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.28;
        
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI, M_PI/2);
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
    }
    transform_vector.push_back(tf_stamped);

    static_tf_broadcaster.sendTransform(transform_vector);

    return true;
}

void TfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_base_cb) { AddTransform(m_base_tf_stamped.header.frame_id, m_base_tf_stamped.child_frame_id, m_base_tf_stamped.transform, transform_vector); m_base_cb = false; }
	
    geometry_msgs::TransformStamped test;

    if (!transform_vector.empty()){
        m_tf_broadcaster.sendTransform(transform_vector);
    } 
}

void TfBroadcaster::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[tf_broadcaster] Home set");
    }
}

void TfBroadcaster::EgoGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr &pos_ptr)
{
    auto lat = pos_ptr->latitude;
    auto lon = pos_ptr->longitude;
    auto alt = m_local_pose.pose.pose.position.z;
    auto pose = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
    auto q = m_local_pose.pose.pose.orientation;
    if (q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0){
        return;
    }

    m_base_tf_stamped.header.frame_id = "map";
    m_base_tf_stamped.child_frame_id = "base_link";

    m_base_tf_stamped.transform.translation.x = pose.position.x;
    m_base_tf_stamped.transform.translation.y = pose.position.y;
    m_base_tf_stamped.transform.translation.z = pose.position.z;

    m_base_tf_stamped.transform.rotation = q;
    
    m_base_cb = true;
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

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "global_tf");
    kuam::TfBroadcaster global_tf;

    ros::spin();

    return 0;
} 