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
    m_camera_alt_m_param(NAN),
    m_data_ns_param("missing")
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
    m_is_experiment_validation_param = false;
}

bool TfBroadcaster::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);
    m_nh.getParam(nd_name + "/is_finding_home", m_is_finding_home_param);
    m_nh.getParam(nd_name + "/target_height_m", m_target_height_m_param);
    m_nh.getParam(nd_name + "/data_ns", m_data_ns_param);
    m_nh.getParam(nd_name + "/drone_offset_m", m_drone_offset_m_param);
    m_nh.getParam(nd_name + "/camera_alt_m", m_camera_alt_m_param);
    m_nh.getParam(nd_name + "/is_experiment_validation", m_is_experiment_validation_param);

    if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_target_height_m_param is NAN"); return false; }
    else if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("[tf_broadcaster] m_data_ns_param is missing"); return false; }
    else if (__isnan(m_drone_offset_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_drone_offset_m_param is NAN"); return false; }
    else if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_process_freq_param is NAN"); return false; }
    else if (__isnan(m_camera_alt_m_param)) { ROS_ERROR_STREAM("[tf_broadcaster] m_camera_alt_m_param is NAN"); return false; }

    return true;
}

void TfBroadcaster::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    
    // Initialize subscriber
    m_novatel_sub = m_nh.subscribe<novatel_oem7_msgs::INSPVA>("/novatel/oem7/inspva", 10, boost::bind(&TfBroadcaster::NovatelINSPVACallback, this, _1));
    m_ego_vehicle_local_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, boost::bind(&TfBroadcaster::EgoVehicleLocalPositionCallback, this, _1));

    if (!m_is_experiment_validation_param){
        if (!m_is_finding_home_param){
            m_home_position_sub = m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&TfBroadcaster::HomePositionCallback, this, _1));
            
            ros::Rate rate(10);
            while (ros::ok() && !m_is_home_set){
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
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
    if (!m_is_experiment_validation_param){
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = 0.1;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI, M_PI/2);
        tf_stamped.transform.rotation.x = q.x();
        tf_stamped.transform.rotation.y = q.y();
        tf_stamped.transform.rotation.z = q.z();
        tf_stamped.transform.rotation.w = q.w();
    }
    else {
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "map";
        tf_stamped.child_frame_id = "camera_link";

        tf_stamped.transform.translation.x = 0.0;
        tf_stamped.transform.translation.y = 0.0;
        tf_stamped.transform.translation.z = m_camera_alt_m_param;
        
        tf2::Quaternion q;
        q.setRPY(0.0, M_PI, M_PI);
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

void TfBroadcaster::NovatelINSPVACallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr)
{
    if (m_is_home_set){
        // geometry_msgs::TransformStamped novatel_tf_stamped;

        // novatel_tf_stamped.header.stamp = inspva_msg_ptr->header.stamp;
        // novatel_tf_stamped.header.frame_id = "map";
        // novatel_tf_stamped.child_frame_id = inspva_msg_ptr->header.frame_id;

        // // Get offset
        // geometry_msgs::Pose novatel_enu_pose = m_utils.ConvertToMapFrame(inspva_msg_ptr->latitude, 
        //                                                                         inspva_msg_ptr->longitude, 
        //                                                                         m_target_height_m_param, 
        //                                                                         m_home_position);
        // novatel_tf_stamped.transform.translation.x = novatel_enu_pose.position.x;
        // novatel_tf_stamped.transform.translation.y = novatel_enu_pose.position.y;
        // novatel_tf_stamped.transform.translation.z = novatel_enu_pose.position.z;
        
        // tf2::Quaternion q;
        // q.setRPY(inspva_msg_ptr->roll * M_PI / 180., inspva_msg_ptr->pitch * M_PI / 180., (-1*inspva_msg_ptr->azimuth + 90.0) * M_PI / 180.);
        // novatel_tf_stamped.transform.rotation.x = q.x();
        // novatel_tf_stamped.transform.rotation.y = q.y();
        // novatel_tf_stamped.transform.rotation.z = q.z();
        // novatel_tf_stamped.transform.rotation.w = q.w();

        // if (!m_novatel_tf_init){
        //     m_transforms.push_back(novatel_tf_stamped);
        // }
        // m_novatel_tf_init = true;
    }
    else if (!m_is_home_set && m_is_finding_home_param){
        m_is_home_set = true;

        m_home_position.latitude = inspva_msg_ptr->latitude;
        m_home_position.longitude = inspva_msg_ptr->longitude;
        m_home_position.altitude = inspva_msg_ptr->height;
        ROS_WARN_STREAM("[tf_broadcaster] Home set");
    }
}

void TfBroadcaster::EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped_ptr)
{
    m_base_cb = true;
    m_base_tf_stamped.header.stamp = pose_stamped_ptr->header.stamp;
    m_base_tf_stamped.header.frame_id = "map";
    m_base_tf_stamped.child_frame_id = "base_link";

    m_base_tf_stamped.transform.translation.x = pose_stamped_ptr->pose.position.x;
    m_base_tf_stamped.transform.translation.y = pose_stamped_ptr->pose.position.y;
    // m_base_tf_stamped.transform.translation.z = pose_stamped_ptr->pose.position.z - m_drone_offset_m_param;
    m_base_tf_stamped.transform.translation.z = pose_stamped_ptr->pose.position.z;

    m_base_tf_stamped.transform.rotation.x = pose_stamped_ptr->pose.orientation.x;
    m_base_tf_stamped.transform.rotation.y = pose_stamped_ptr->pose.orientation.y;
    m_base_tf_stamped.transform.rotation.z = pose_stamped_ptr->pose.orientation.z;
    m_base_tf_stamped.transform.rotation.w = pose_stamped_ptr->pose.orientation.w;
}

void TfBroadcaster::MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr)
{
    if (marker_ptr->transforms.size() > 0){
        m_marker_cb = true;
        m_marker_tf_stamped = marker_ptr->transforms[0]; // After get multiple aruco marker, change to select mode

        tf::Quaternion ego_q(
        m_base_tf_stamped.transform.rotation.x,
        m_base_tf_stamped.transform.rotation.y,
        m_base_tf_stamped.transform.rotation.z,
        m_base_tf_stamped.transform.rotation.w);
        
        tf::Matrix3x3 ego_m(ego_q);
        double ego_roll, ego_pitch, ego_yaw;
        ego_m.getRPY(ego_roll, ego_pitch, ego_yaw);

        tf::Quaternion marker_q(
        m_marker_tf_stamped.transform.rotation.x,
        m_marker_tf_stamped.transform.rotation.y,
        m_marker_tf_stamped.transform.rotation.z,
        m_marker_tf_stamped.transform.rotation.w);
        
        tf::Matrix3x3 marker_m(marker_q);
        double marker_roll, marker_pitch, marker_yaw;
        marker_m.getRPY(marker_roll, marker_pitch, marker_yaw);
        double tmp_r = marker_roll;
        double tmp_p = marker_pitch;
        double tmp_w = marker_yaw;
        marker_roll -= ego_roll;
        marker_pitch -= ego_pitch;
        marker_yaw -= ego_yaw;

        tf2::Quaternion q;
        q.setRPY(marker_roll, marker_pitch, marker_yaw);
        m_marker_tf_stamped.transform.rotation.x = q.x();
        m_marker_tf_stamped.transform.rotation.y = q.y();
        m_marker_tf_stamped.transform.rotation.z = q.z();
        m_marker_tf_stamped.transform.rotation.w = q.w();
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