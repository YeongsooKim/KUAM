#ifndef __VEHICLE_POSITION_H__
#define __VEHICLE_POSITION_H__

#include <ros/ros.h>
#include <string>
#include <vector>

// Messages
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <kuam_msgs/VehicleState.h>

// Tf
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

namespace kuam{

struct BoundingBox{
    double x;
    double y;
    double w;
    double l;
};

class VehiclePosition
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

public:
    VehiclePosition();
    ~VehiclePosition();

private:
    // Subscriber
    ros::Subscriber m_bounding_box_sub;

    // Publisher
    ros::Publisher m_vehicle_state_pub;

    // Timer
    ros::Timer m_process_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    float m_target_width_m_param;
    float m_target_length_m_param;
    float m_image_plane_width_m_param;
    float m_image_plane_height_m_param;
    float m_focal_length_param;

    // Const value
    // starex size: width - 1920, length - 5150 mm, diag - 5496.26 (1 : 2.68 : 2.86)
    const double C1 = 2.68;
    const double C2 = 2.86;
    const double STAREX_WIDTH = 1.92;
    const double STAREX_LENGTH = 5.15;

    // tf
    tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

    // Variable
    vector<BoundingBox> m_bounding_boxes;
    BoundingBox m_vehicle;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();

    // Calculate bounding box width and height in image plane
    // Normalize image plane coordinate (optical axis is origin)
    // Calculate target vehicle position in normalized image plane
    void ProcessTimerCallback(const ros::TimerEvent& event);
    void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr bounding_box_ptr);
    
    geometry_msgs::Quaternion GetOrientation(geometry_msgs::Point p1, geometry_msgs::Point p2);
};
}

#endif // __VEHICLE_POSITION_H__