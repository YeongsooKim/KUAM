#ifndef __TF_BROADCASTER_H__
#define __TF_BROADCASTER_H__


#include "ros/ros.h"
#include <ros/spinner.h>

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
#include <novatel_oem7_msgs/INSPVA.h>

#include <sensor_msgs/Imu.h>

#include "kuam_mission_manager/utils.h"

namespace kuam{

class TfBroadcaster{
private:
	ros::NodeHandle m_nh;
    Utils m_utils;

public:
    TfBroadcaster();
    ~TfBroadcaster();

private:
    // Subscriber
    ros::Subscriber m_novatel_sub;
    ros::Subscriber m_ego_vehicle_imu_sub;
    ros::Subscriber m_ego_vehicle_local_pose_sub;
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
    float m_target_height_m_param;
    std::string m_data_ns_param;
    float m_drone_offset_m_param;
    float m_camera_alt_m_param;
    bool m_is_experiment_validation_param;

    // Flag
    bool m_base_cb;
    bool m_marker_cb;

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
    void NovatelINSPVACallback(const novatel_oem7_msgs::INSPVA::ConstPtr &inspva_msg_ptr);
    void EgoVehicleLocalPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_stamped_ptr);
    void MarkerCallback(const tf2_msgs::TFMessage::ConstPtr &marker_ptr);
    
    void AddTransform(const std::string &frame_id, const std::string &child_id, const geometry_msgs::Transform tf, std::vector<geometry_msgs::TransformStamped>& vector);
};
}

#endif // __TF_BROADCASTER_H__