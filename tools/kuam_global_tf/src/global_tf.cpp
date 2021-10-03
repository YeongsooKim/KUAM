#include <math.h>
#include <Eigen/Dense>

// Messages
#include <geometry_msgs/Pose.h>

#include "kuam_global_tf/global_tf.h"

using namespace std;
using namespace kuam;

TFBroadcaster::TFBroadcaster() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[global_tf] Fail GetParam %s", m_err_param.c_str());
    InitROS();
    InitStaticTf();
}

TFBroadcaster::~TFBroadcaster()
{}

bool TFBroadcaster::InitFlag()
{
    return true;
}

bool TFBroadcaster::GetParam()
{
    if (!m_p_nh.getParam("data_ns", m_data_ns_param)) { m_err_param = "data_ns"; return false; }
    if (!m_nh.getParam(m_data_ns_param + "/aruco_tracking/camera_frame_id", m_camera_frame_id_param)) { m_err_param = "camera_frame_id"; return false; }
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("is_real", m_is_real_param)) { m_err_param = "is_real"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_x_m", m_extrinsic_imu_to_camera_x_m_param)) { m_err_param = "extrinsic_imu_to_camera_x_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_y_m", m_extrinsic_imu_to_camera_y_m_param)) { m_err_param = "extrinsic_imu_to_camera_y_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_z_m", m_extrinsic_imu_to_camera_z_m_param)) { m_err_param = "extrinsic_imu_to_camera_z_m"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_r_deg", m_extrinsic_imu_to_camera_r_deg_param)) { m_err_param = "extrinsic_imu_to_camera_r_deg"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_p_deg", m_extrinsic_imu_to_camera_p_deg_param)) { m_err_param = "extrinsic_imu_to_camera_p_deg"; return false; }
    if (!m_p_nh.getParam("extrinsic_imu_to_camera_y_deg", m_extrinsic_imu_to_camera_y_deg_param)) { m_err_param = "extrinsic_imu_to_camera_y_deg"; return false; }

    return true;
}

bool TFBroadcaster::InitROS()
{
    return true;
}

bool TFBroadcaster::InitStaticTf()
{
    static tf2_ros::StaticTransformBroadcaster static_tf_broadcaster;
	vector<geometry_msgs::TransformStamped> transform_vector;

    geometry_msgs::TransformStamped tf_stamped;

    // camera_link static transformation
    if (m_is_real_param){
        tf_stamped.header.stamp = ros::Time::now();
        tf_stamped.header.frame_id = "base_link";
        tf_stamped.child_frame_id = m_camera_frame_id_param;

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
        tf_stamped.child_frame_id = m_camera_frame_id_param;

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

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "global_tf");
    kuam::TFBroadcaster global_tf;

    ros::spin();

    return 0;
} 