#include <kuam_aruco_tracking/utils/util_setpoint.h>
#include <sstream>
#include <math.h>

namespace kuam
{
UtilSetpoint::UtilSetpoint() :
    DEG2RAD(M_PI/180.0),
    RAD2DEG(180.0/M_PI)
{}

UtilSetpoint::~UtilSetpoint() {}
    
void UtilSetpoint::Translate(geometry_msgs::Pose& pose, const int id, 
                            const float big_marker_trans, const float medium_marker_trans, const float small_marker_trans)
{
    auto theta_rad = GetYawRad(pose.orientation);

    float x_trans = 0;
    float y_trans = 0;

    if (id == 0) { x_trans = +big_marker_trans; y_trans = -big_marker_trans; }
    else if (id == 1) { x_trans = 0.0; y_trans = -big_marker_trans; }
    else if (id == 2) { x_trans = -big_marker_trans; y_trans = -big_marker_trans; }
    else if (id == 3) { x_trans = -big_marker_trans; y_trans = 0.0; }
    else if (id == 4) { x_trans = -big_marker_trans; y_trans = 0.0; }
    else if (id == 5) { x_trans = 0.0; y_trans = +big_marker_trans; }
    else if (id == 6) { x_trans = +big_marker_trans; y_trans = +big_marker_trans; }
    else if (id == 7) { x_trans = +big_marker_trans; y_trans = +big_marker_trans; }
    else if (id == 8) { x_trans = +medium_marker_trans; y_trans = -medium_marker_trans; }
    else if (id == 9) { x_trans = 0.0; y_trans = -medium_marker_trans; }
    else if (id == 10) { x_trans = -medium_marker_trans; y_trans = -medium_marker_trans; }
    else if (id == 11) { x_trans = -medium_marker_trans; y_trans = 0.0; }
    else if (id == 12) { x_trans = -medium_marker_trans; y_trans = 0.0; }
    else if (id == 13) { x_trans = 0.0; y_trans = +medium_marker_trans; }
    else if (id == 14) { x_trans = +medium_marker_trans; y_trans = +medium_marker_trans; }
    else if (id == 15) { x_trans = +medium_marker_trans; y_trans = +medium_marker_trans; }
    else if (id == 16) { x_trans = 0.0; y_trans = 0.0; }

    pose.position.x += (x_trans*cos(theta_rad) + y_trans*sin(theta_rad));
    pose.position.y += (x_trans*sin(theta_rad) - y_trans*cos(theta_rad));
}

float UtilSetpoint::GetYawRad(const geometry_msgs::Quaternion& quat_msg)
{
	tf2::Quaternion quat_tf(
        quat_msg.x,
        quat_msg.y,
        quat_msg.z,
        quat_msg.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
	
    return yaw;
}

float UtilSetpoint::GetYawDeg(const geometry_msgs::Quaternion& quat_msg)
{
	tf2::Quaternion quat_tf(
        quat_msg.x,
        quat_msg.y,
        quat_msg.z,
        quat_msg.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
	
    return yaw*RAD2DEG;
}

bool UtilSetpoint::IsValid(geometry_msgs::Pose pose)
{
    auto x = pose.position.x;
    auto y = pose.position.y;
    auto z = pose.position.z;

    if ((x>1e+100) || (x<-1e+100)) return false;
    if ((y>1e+100) || (y<-1e+100)) return false;
    if ((z>1e+100) || (z<-1e+100)) return false;
    
    return true;
}

geometry_msgs::Pose UtilSetpoint::GetSetpoint(vector<geometry_msgs::Pose> poses)
{
    float sum_x = 0.0; 
    float sum_y = 0.0;
    float sum_z = 0.0;
    float sum_yaw_deg = 0.0;
    for (auto p : poses){
        sum_x += p.position.x;
        sum_y += p.position.y;
        sum_z += p.position.z;
        sum_yaw_deg += GetYawDeg(p.orientation);
    }

    geometry_msgs::Pose p;
    float size = poses.size();
    p.position.x = sum_x/size;
    p.position.y = sum_y/size;
    p.position.z = sum_z/size;

    float avg_yaw_deg = sum_yaw_deg/size;
    tf2::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, avg_yaw_deg*DEG2RAD);
    p.orientation.x = q_tf.x();
    p.orientation.y = q_tf.y();
    p.orientation.z = q_tf.z();
    p.orientation.w = q_tf.w();

    return p;
}

// bool UtilSetpoint::MovingAvgFilter(const Eigen::Vector3d pos)
// {
//     // Initialize
//     if (!m_is_init_maf){
//         for (auto& p : m_pos_buf){
//             p = pos;
//         }

//         m_is_init_maf = true;
//     }

//     // Shift
//     for (int i = 0; i < MAF_BUF_SIZE - 1; i++){
//         m_pos_buf.at(i) = m_pos_buf.at(i+1);
//     }
//     m_pos_buf.at(MAF_BUF_SIZE - 1) = pos;

//     // Summation
//     Eigen::Vector3d sum;
//     sum << 0, 0, 0;
//     for (auto p : m_pos_buf){
//         sum += p;
//     }

//     // Average
//     m_state[(int)EstimatingMethod::MAF].position = sum/(double)MAF_BUF_SIZE;
//     m_state[(int)EstimatingMethod::MAF].prev_position = pos;
// }

}