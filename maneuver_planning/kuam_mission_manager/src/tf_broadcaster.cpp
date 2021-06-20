#include "kuam_mission_manager/tf_broadcaster.h"
#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

using namespace std;
using namespace kuam;

TfBroadcaster::TfBroadcaster() :
    m_process_freq_param(NAN),
    m_target_height_m_param(NAN),
    m_drone_offset_m_param(NAN),
    m_exp_camera_height_m_param(NAN),
    m_data_ns_param("missing"),
    m_extrinsic_imu_to_camera_x_param(NAN),
    m_extrinsic_imu_to_camera_y_param(NAN),
    m_extrinsic_imu_to_camera_z_param(NAN)    
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("Fail GetParam");
    InitROS();
    InitStaticTf();
}

TfBroadcaster::~TfBroadcaster()
{}

void TfBroadcaster::InitFlag()
{
    m_is_home_set = false;
    m_base_cb = false;
    m_marker_cb = false;
    m_is_exp_param = false;
}

bool TfBroadcaster::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam("/data_ns", m_data_ns_param);
    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);
    m_nh.getParam(nd_name + "/is_finding_home", m_is_finding_home_param);
    m_nh.getParam(nd_name + "/target_height_m", m_target_height_m_param);
    m_nh.getParam(nd_name + "/drone_offset_m", m_drone_offset_m_param);
    m_nh.getParam(nd_name + "/is_exp", m_is_exp_param);
    m_nh.getParam(nd_name + "/is_real", m_is_real_param);
    m_nh.getParam(nd_name + "/is_gazebo", m_is_gazebo_param);
    m_nh.getParam(nd_name + "/exp_camera_height_m", m_exp_camera_height_m_param);
    m_nh.getParam(nd_name + "/extrinsic_imu_to_camera_x", m_extrinsic_imu_to_camera_x_param);
    m_nh.getParam(nd_name + "/extrinsic_imu_to_camera_y", m_extrinsic_imu_to_camera_y_param);
    m_nh.getParam(nd_name + "/extrinsic_imu_to_camera_z", m_extrinsic_imu_to_camera_z_param);

    if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_target_height_m_param is NAN"); return false; }
    else if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("[tf_broadcaster] m_data_ns_param is missing"); return false; }
    else if (__isnan(m_drone_offset_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_drone_offset_m_param is NAN"); return false; }
    else if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_process_freq_param is NAN"); return false; }
    else if (__isnan(m_exp_camera_height_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_exp_camera_height_m_param is NAN"); return false; }
    else if (__isnan(m_extrinsic_imu_to_camera_x_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_extrinsic_imu_to_camera_x_param is NAN"); return false; }
    else if (__isnan(m_extrinsic_imu_to_camera_y_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_extrinsic_imu_to_camera_y_param is NAN"); return false; }
    else if (__isnan(m_extrinsic_imu_to_camera_z_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_extrinsic_imu_to_camera_z_param is NAN"); return false; }

    return true;
}

void TfBroadcaster::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    
    // Initialize subscriber
    if (!m_is_exp_param){
        if (!m_is_finding_home_param){
            m_home_position_sub = m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&TfBroadcaster::HomePositionCallback, this, _1));
            
            ros::Rate rate(10);
            while (ros::ok() && !m_is_home_set){
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
    m_local_pose_sub = m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&TfBroadcaster::EgoLocalCallback, this, _1));
    m_ego_global_pos_sub = m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&TfBroadcaster::EgoGlobalCallback, this, _1));
    m_aruco_sub = m_nh.subscribe<tf2_msgs::TFMessage>(m_data_ns_param + "/aruco_tracking/tf_list", 10, boost::bind(&TfBroadcaster::MarkerCallback, this, _1));

    // Initialize publisher
    m_home_position_pub = m_nh.advertise<geographic_msgs::GeoPoint>(nd_name + "/home", 1);

    // Initialize timer
    m_home_position_timer = m_nh.createTimer(ros::Duration(2.0), &TfBroadcaster::HomePositionTimerCallback, this);
    m_tf_broadcaster_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &TfBroadcaster::ProcessTimerCallback, this);
}


void TfBroadcaster::InitStaticTf(void)
{
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> transform_vector;

    geometry_msgs::TransformStamped tf_stamped;

    // odom static transformation
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = "map_ned";
    tf_stamped.child_frame_id = "odom";

    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    
    tf_stamped.transform.rotation.x = 0.0;
    tf_stamped.transform.rotation.y = 0.0;
    tf_stamped.transform.rotation.z = 0.0;
    tf_stamped.transform.rotation.w = 1.0;

    transform_vector.push_back(tf_stamped);

    // camera_link static transformation
    if (m_is_gazebo_param){
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
    else if (m_is_exp_param){
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "map";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = m_exp_camera_height_m_param;
        
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI, M_PI/2);
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
    }
    else if (m_is_real_param){
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = m_extrinsic_imu_to_camera_x_param;
        tf_stamped.transform.translation.y = m_extrinsic_imu_to_camera_y_param;
        tf_stamped.transform.translation.z = m_extrinsic_imu_to_camera_z_param;
        
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI, M_PI/2);
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
    }
    transform_vector.push_back(tf_stamped);

    static_tf_broadcaster.sendTransform(transform_vector);
}


void TfBroadcaster::HomePositionTimerCallback(const ros::TimerEvent& event)
{
    if (m_is_home_set){
        m_home_position_pub.publish(m_home_position);
        
        if (m_is_finding_home_param){
            ROS_INFO("---");
            ROS_INFO("Home latitude: %f", m_home_position.latitude);
            ROS_INFO("Home longitude: %f", m_home_position.longitude);
            ROS_INFO("Home altitude: %f", m_home_position.altitude);
        }
    }
}

void TfBroadcaster::ProcessTimerCallback(const ros::TimerEvent& event)
{
	std::vector<geometry_msgs::TransformStamped> transform_vector;
    if (m_base_cb) { AddTransform(m_base_tf_stamped.header.frame_id, m_base_tf_stamped.child_frame_id, m_base_tf_stamped.transform, transform_vector); m_base_cb = false; }
    if (m_marker_cb) { AddTransform(m_marker_tf_stamped.header.frame_id, m_marker_tf_stamped.child_frame_id, m_marker_tf_stamped.transform, transform_vector); m_marker_cb = false; }
	
    m_tf_broadcaster.sendTransform(transform_vector);
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
    m_base_cb = true;

    auto lat = pos_ptr->latitude;
    auto lon = pos_ptr->longitude;
    auto alt = m_local_pose.pose.pose.position.z;
    auto pose = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
    auto q = m_local_pose.pose.pose.orientation;

    m_base_tf_stamped.header.frame_id = "map";
    m_base_tf_stamped.child_frame_id = "base_link";

    m_base_tf_stamped.transform.translation.x = pose.position.x;
    m_base_tf_stamped.transform.translation.y = pose.position.y;
    m_base_tf_stamped.transform.translation.z = pose.position.z;

    m_base_tf_stamped.transform.rotation = q;
}

void TfBroadcaster::MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr)
{
    if (marker_ptr->transforms.size() > 0){
        m_marker_cb = true;
        m_marker_tf_stamped = marker_ptr->transforms[0];

        tf::Quaternion marker_q(
        m_marker_tf_stamped.transform.rotation.x,
        m_marker_tf_stamped.transform.rotation.y,
        m_marker_tf_stamped.transform.rotation.z,
        m_marker_tf_stamped.transform.rotation.w);
    }
}

void TfBroadcaster::AddTransform(const std::string &frame_id, const std::string &child_id, const geometry_msgs::Transform tf, std::vector<geometry_msgs::TransformStamped>& vector)
{
	geometry_msgs::TransformStamped tf_stamped;

	tf_stamped.header.stamp = ros::Time::now();
	tf_stamped.header.frame_id = frame_id;
	tf_stamped.child_frame_id = child_id;
    tf_stamped.transform = tf;

	vector.push_back(tf_stamped);
}
