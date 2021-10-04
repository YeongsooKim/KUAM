#ifndef __STATE_MACHINE_TF_H__
#define __STATE_MACHINE_TF_H__


#include "ros/ros.h"
#include <ros/spinner.h>
#include <string>
#include <vector>

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

using namespace std; 

namespace kuam{
class SetpointTfBroadcaster{
private:
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

public:
    SetpointTfBroadcaster();
    ~SetpointTfBroadcaster();

private:
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

private:
    bool InitFlag();
    bool GetParam();
    bool InitROS();
    bool InitStaticTf();
};
}

#endif // __STATE_MACHINE_TF_H__