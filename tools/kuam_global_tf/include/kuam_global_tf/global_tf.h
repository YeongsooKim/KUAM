#ifndef __GLOBAL_TF_H__
#define __GLOBAL_TF_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>
#include <vector>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include <tf/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>

#include <kuam_global_tf/utils.h>

using namespace std; 

namespace kuam{
class TfBroadcaster{
private:
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

    Utils m_utils;

public:
    TfBroadcaster();
    ~TfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_ego_global_pos_sub;
    ros::Subscriber m_local_pose_sub;

    // Publisher
    // ros::Publisher m_payload_cmd_pub;
    
    // Timer
    ros::Timer m_tf_broadcaster_timer;

    // Flag
    bool m_is_home_set;
    bool m_base_cb;

    // Param
    string m_err_param;
    float m_process_freq_param;
    bool m_is_real_param;
    string m_camera_frame_id_param;
    float m_extrinsic_imu_to_camera_x_m_param;
    float m_extrinsic_imu_to_camera_y_m_param;
    float m_extrinsic_imu_to_camera_z_m_param;
    float m_extrinsic_imu_to_camera_r_deg_param;
    float m_extrinsic_imu_to_camera_p_deg_param;
    float m_extrinsic_imu_to_camera_y_deg_param;
    bool m_is_sim_param;

    geographic_msgs::GeoPoint m_home_position;
    geometry_msgs::TransformStamped m_base_tf_stamped;
    nav_msgs::Odometry m_local_pose;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;

private:
    bool InitFlag();
    bool GetParam();
    bool InitROS();
    bool InitStaticTf();

    void ProcessTimerCallback(const ros::TimerEvent& event);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void EgoGlobalCallback(const sensor_msgs::NavSatFix::ConstPtr &pos_ptr);
    inline void EgoLocalCallback(const nav_msgs::Odometry::ConstPtr &pose_ptr) { m_local_pose = *pose_ptr; }

    void AddTransform(const string &frame_id, const string &child_id, const geometry_msgs::Transform tf, vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __GLOBAL_TF_H__