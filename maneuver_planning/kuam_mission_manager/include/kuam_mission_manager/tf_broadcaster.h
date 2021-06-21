#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>

#include <mavros_msgs/HomePosition.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "kuam_mission_manager/utils.h"

using namespace std; 

namespace kuam{
const string CAMERA_FRAME = "head_camera";
class TfBroadcaster{
private:
	ros::NodeHandle m_nh;
    Utils m_utils;

public:
    TfBroadcaster();
    ~TfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_ego_global_pos_sub;
    ros::Subscriber m_local_pose_sub;
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_aruco_sub;

    // Publisher
    ros::Publisher m_home_position_pub;

    // Timer
    ros::Timer m_home_position_timer;
    ros::Timer m_tf_broadcaster_timer;

    // Param
    float m_process_freq_param;
    bool m_is_finding_home_param;
    std::string m_data_ns_param;
    bool m_is_exp_param;
    bool m_is_real_param;
    bool m_is_gazebo_param;
    float m_exp_camera_height_m_param;
    float m_extrinsic_imu_to_camera_x_param;
    float m_extrinsic_imu_to_camera_y_param;
    float m_extrinsic_imu_to_camera_z_param;

    // Flag
    bool m_base_cb;
    bool m_marker_cb;

    nav_msgs::Odometry m_local_pose;
    geometry_msgs::TransformStamped m_base_tf_stamped;
    geometry_msgs::TransformStamped m_marker_tf_stamped;
    geographic_msgs::GeoPoint m_home_position;
    bool m_is_home_set;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;
	std::vector<geometry_msgs::TransformStamped> m_transforms;

private:
    void InitFlag();
    bool GetParam();
    void InitROS();
    void InitStaticTf();

    void HomePositionTimerCallback(const ros::TimerEvent& event);
    void ProcessTimerCallback(const ros::TimerEvent& event);

    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    inline void EgoLocalCallback(const nav_msgs::Odometry::ConstPtr &pose_ptr) { m_local_pose = *pose_ptr; }
    void EgoGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr &pos_ptr);
    void MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr);
    
    void AddTransform(const std::string &frame_id, const std::string &child_id, const geometry_msgs::Transform tf, std::vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __TF_BROADCASTER_H__
