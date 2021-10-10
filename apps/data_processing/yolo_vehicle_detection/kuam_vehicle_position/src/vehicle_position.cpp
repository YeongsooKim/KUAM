#include <cmath>
#include "kuam_vehicle_position/vehicle_position.h"

using namespace std;

namespace kuam{
    
VehiclePosition::VehiclePosition() :
    m_p_nh("~"),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_tracking] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

VehiclePosition::~VehiclePosition()
{
}

bool VehiclePosition::InitFlag()
{
    return true;
}

bool VehiclePosition::GetParam()
{
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("target_width_m", m_target_width_m_param)) { m_err_param = "target_width_m"; return false; }
    if (!m_p_nh.getParam("target_length_m", m_target_length_m_param)) { m_err_param = "target_length_m"; return false; }
    if (!m_p_nh.getParam("image_plane_width_m", m_image_plane_width_m_param)) { m_err_param = "image_plane_width_m"; return false; }
    if (!m_p_nh.getParam("image_plane_height_m", m_image_plane_height_m_param)) { m_err_param = "image_plane_height_m"; return false; }
    if (!m_p_nh.getParam("focal_length", m_focal_length_param)) { m_err_param = "focal_length"; return false; }

    return true;
}

bool VehiclePosition::InitROS()
{
    // Initialize subscriber
    m_bounding_box_sub = m_nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, boost::bind(&VehiclePosition::BoundingBoxCallback, this, _1));
    m_local_pose_sub = m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&VehiclePosition::EgoLocalCallback, this, _1));
    
    // Initialize publisher
    m_vehicle_state_pub = m_p_nh.advertise<kuam_msgs::VehicleState>("vehicle_state", 10);
    
    // Initialize timer
    m_process_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &VehiclePosition::ProcessTimerCallback, this);

    return true;
}

void VehiclePosition::ProcessTimerCallback(const ros::TimerEvent& event)
{
}

void VehiclePosition::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr bounding_box_ptr)
{
    if (bounding_box_ptr->bounding_boxes.empty())
        return;

    // Calculate center point of bounding box
    vector<Center> bounding_boxes;
    bounding_boxes.clear();
    for (auto bb : bounding_box_ptr->bounding_boxes){
        auto dx = bb.xmax - bb.xmin;
        auto dy = bb.ymax - bb.ymin;

        Center c;
        c.x = dx/2.0 + bb.xmin;
        c.y = dy/2.0 + bb.ymin;
        bounding_boxes.push_back(c);
    }

    // Normalized image plane coordinate (optical axis is origin)
    vector<Center> norm_bounding_boxes;
    for (auto bb : bounding_boxes){
        Center c;
        c.x = bb.x - m_image_plane_width_m_param/2.0;
        c.y = -bb.y + m_image_plane_height_m_param/2.0;

        norm_bounding_boxes.push_back(c);
    }

    // Calculate relative position relation to camera origin in camera_link
    double sum_x = 0.0; double sum_y = 0.0;
    auto ratio = m_ego_point.z / m_focal_length_param;
    for (auto bb : norm_bounding_boxes){
        sum_x += (bb.x * ratio);
        sum_y += (bb.y * ratio);
    }

    geometry_msgs::Pose vehicle;
    vehicle.position.x = sum_x / norm_bounding_boxes.size();
    vehicle.position.y = -sum_y / norm_bounding_boxes.size();
    vehicle.position.z = m_ego_point.z;
    vehicle.orientation.x = 0.0;
    vehicle.orientation.y = 0.0;
    vehicle.orientation.z = 0.0;
    vehicle.orientation.w = 1.0;
    // vehicle.orientation = GetOrientation(m_ego_point, vehicle.position);

    kuam_msgs::VehicleState vehicle_state;
    vehicle_state.header.frame_id = "camera_link";
    vehicle_state.is_detected = true;
    vehicle_state.vehicle = vehicle;
    vehicle_state.focal_length = m_focal_length_param;

    // Publish
    m_vehicle_state_pub.publish(vehicle_state);
}


void VehiclePosition::EgoLocalCallback(const nav_msgs::Odometry::ConstPtr local_ptr)
{
    auto p = local_ptr->pose.pose;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::Pose transformed_pose;
    try{
        transformStamped = m_tfBuffer.lookupTransform("camera_link", "base_link", ros::Time(0));
        tf2::doTransform(p, transformed_pose, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
    }

    m_ego_point = p.position;
}



geometry_msgs::Quaternion VehiclePosition::GetOrientation(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    auto dx = p1.x - p2.x;
    auto dy = p1.y - p2.y;
    auto yaw_rad = atan2(dy, dx);

    tf2::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, yaw_rad);
    
    geometry_msgs::Quaternion q;
    q.x = q_tf.x();
    q.y = q_tf.y();
    q.z = q_tf.z();
    q.w = q_tf.w();

    return q;
}
}