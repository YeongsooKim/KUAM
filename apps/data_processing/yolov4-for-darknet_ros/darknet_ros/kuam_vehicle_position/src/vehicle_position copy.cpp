#include <cmath>
#include "kuam_vehicle_position/vehicle_position.h"

using namespace std;

namespace kuam{
    
VehiclePosition::VehiclePosition() :
    m_p_nh("~")
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
    
    // Initialize publisher
    m_vehicle_states_pub = m_p_nh.advertise<kuam_msgs::VehicleStates>("vehicle_states", 10);
    
    // Initialize timer
    m_process_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &VehiclePosition::ProcessTimerCallback, this);

    return true;
}

void VehiclePosition::ProcessTimerCallback(const ros::TimerEvent& event)
{
}

void VehiclePosition::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr bounding_box_ptr)
{
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

    // Pixel relative distance
    vector<Center> distances;
    double x_sum = 0.0; double y_sum = 0.0;;
    for (auto bb : norm_bounding_boxes){
        geometry_msgs::Point p;
        x_sum += bb.x;
        y_sum += bb.y;
    }

    geometry_msgs::Point vehicle;
    vehicle.x = x_sum/norm_bounding_boxes.size();
    vehicle.y = y_sum/norm_bounding_boxes.size();
    vehicle.z = 0.0;

    kuam_msgs::VehicleStates vehicle_states;
    vehicle_states.header.frame_id = "camera_link";
    vehicle_states.is_detected = true;
    vehicle_states.vehicle = vehicle;
    vehicle_states.focal_length = m_focal_length_param;

    m_vehicle_states_pub.publish(vehicle_states);
}

// tf2::Quaternion VehiclePosition::GetOrientation(geometry_msgs::Point p1, geometry_msgs::Point p2)
// {
//     auto dx = p1.x - p2.x;
//     auto dy = p1.y - p2.y;
//     auto yaw_rad = atan2(dy, dx);

//     tf2::Quaternion q_tf;
//     q_tf.setRPY(0.0, 0.0, yaw_rad);
//     p.orientation.x = q_tf.x();
//     p.orientation.y = q_tf.y();
//     p.orientation.z = q_tf.z();
//     p.orientation.w = q_tf.w();

//     return q_tf;
// }
}