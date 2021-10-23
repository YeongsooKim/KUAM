#include <kuam_aruco_tracking/utils/util_setpoint.h>
#include <Eigen/Dense>
#include <sstream>
#include <math.h>

namespace kuam
{
UtilSetpoint::UtilSetpoint() :
    DEG2RAD(M_PI/180.0),
    RAD2DEG(180.0/M_PI)
{}

UtilSetpoint::~UtilSetpoint() {}
    
// 
void UtilSetpoint::Transform(geometry_msgs::Pose& pose, const int id, 
                            const float big_marker_trans, const float medium_marker_trans, const float small_marker_trans)
{
    auto theta_rad = GetYawRad(pose.orientation);

    float x_trans = 0;
    float y_trans = 0;

    if (id == 0)       { x_trans = +big_marker_trans;        y_trans = -big_marker_trans; }
    else if (id == 1)  { x_trans = 0.0;                      y_trans = -big_marker_trans; }
    else if (id == 2)  { x_trans = -big_marker_trans;        y_trans = -big_marker_trans; }
    else if (id == 3)  { x_trans = -big_marker_trans;        y_trans = 0.0; }
    else if (id == 4)  { x_trans = -big_marker_trans;        y_trans = +big_marker_trans; }
    else if (id == 5)  { x_trans = 0.0;                      y_trans = +big_marker_trans; }
    else if (id == 6)  { x_trans = +big_marker_trans;        y_trans = +big_marker_trans; }
    else if (id == 7)  { x_trans = +big_marker_trans;        y_trans = 0.0; }
    else if (id == 8)  { x_trans = +medium_marker_trans;     y_trans = -medium_marker_trans; }
    else if (id == 9)  { x_trans = 0.0;                      y_trans = -medium_marker_trans; }
    else if (id == 10) { x_trans = -medium_marker_trans;     y_trans = -medium_marker_trans; }
    else if (id == 11) { x_trans = -medium_marker_trans;     y_trans = 0.0; }
    else if (id == 12) { x_trans = -medium_marker_trans;     y_trans = +medium_marker_trans; }
    else if (id == 13) { x_trans = 0.0;                      y_trans = +medium_marker_trans; }
    else if (id == 14) { x_trans = +medium_marker_trans;     y_trans = +medium_marker_trans; }
    else if (id == 15) { x_trans = +medium_marker_trans;     y_trans = 0.0; }
    else if (id == 16) { x_trans = +small_marker_trans;      y_trans = +small_marker_trans; }
    else if (id == 17) { x_trans = -small_marker_trans;      y_trans = +small_marker_trans; }
    else if (id == 18) { x_trans = -small_marker_trans;      y_trans = -small_marker_trans; }
    else if (id == 19) { x_trans = +small_marker_trans;      y_trans = -small_marker_trans; }

    pose.position.x += (x_trans*cos(theta_rad) + y_trans*sin(theta_rad));
    pose.position.y += (x_trans*sin(theta_rad) - y_trans*cos(theta_rad));
}

geometry_msgs::Pose UtilSetpoint::GetSetpoint(vector<geometry_msgs::Pose> poses)
{
    // Get marker poses and fit plane
    // Compare fitting plane's normal vector and z axis
    // When the angle between normal vector and z axis is less than threshold update target state (is_detected, pose)
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
    q_tf.setRPY(M_PI, 0.0, avg_yaw_deg*DEG2RAD);
    p.orientation.x = q_tf.x();
    p.orientation.y = q_tf.y();
    p.orientation.z = q_tf.z();
    p.orientation.w = q_tf.w();

    return p;
}


bool UtilSetpoint::GeneratePlane(const vector<kuam_msgs::ArucoState>& markers, 
                            geometry_msgs::PoseStamped& plane, Z2NormalAngle& z_to_normal, 
                            const double& plane_threshold, const int& iterations, const double& angle_threshold, bool& is_valid)
{
    // Plane fitting
    std::vector<Vector3VP> points;
    double *center = new double[3];
    double *coefs = new double[4];

    for (auto marker : markers){
        double x = marker.pose.position.x;
        double y = marker.pose.position.y;
        double z = marker.pose.position.z;
        Vector3VP Pt3d = {x, y, z};
        
        points.push_back(Pt3d);
    }
    if (!PlaneFitting(points, center, coefs, plane_threshold, iterations)){
        return false;
    }
    
    // Create a quaternion for rotation into XY plane
    Eigen::Vector3f current(coefs[0], coefs[1], coefs[2]);
    Eigen::Vector3f target(0.0, 0.0, 1.0);
    Eigen::Quaternion<float> q;
    q.setFromTwoVectors(current, target);

    plane.header.frame_id = "camera_link";
    plane.header.stamp = ros::Time::now();
    plane.pose.position.x = center[0];
    plane.pose.position.y = center[1];
    plane.pose.position.z = center[2];
    plane.pose.orientation.x = q.x();
    plane.pose.orientation.y = q.y();
    plane.pose.orientation.z = q.z();
    plane.pose.orientation.w = q.w();

    // Calculate angle between z axis and fitted plane normal vector 
    auto s = sqrt(pow(coefs[0], 2.0) + pow(coefs[1], 2.0) + pow(coefs[2], 2.0));
    auto angle_deg = acos(1/s)*180.0/M_PI;
    MovingAvgFilter(z_to_normal, angle_deg);
    if (z_to_normal.angle_deg < angle_threshold){
        is_valid = true;
    }
    
    return true;
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

bool UtilSetpoint::PlaneFitting(const std::vector<Vector3VP> &points_input, double* center, double* normal, double threshold, double iterations)
{
	int Num = points_input.size();
	std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
	CandPoints.resize(Num);
#pragma omp parallel for num_threads(6)
	for (int i = 0; i <Num; ++i)
	{
		Vector3VP p=points_input[i];
		std::shared_ptr<GRANSAC::AbstractParameter> CandPt = std::make_shared<Point3D>(p[0], p[1],p[2]);
		CandPoints[i]=CandPt;
	}
	
	GRANSAC::RANSAC<PlaneModel, 3> Estimator;
    Estimator.Initialize(threshold, iterations); // Threshold, iterations

    int64_t start = cv::getTickCount();
	Estimator.Estimate(CandPoints);
    int64_t end = cv::getTickCount();
    std::cout << "RANSAC took: " << GRANSAC::VPFloat(end - start) / GRANSAC::VPFloat(cv::getTickFrequency()) * 1000.0 << " ms." << std::endl;
	
	auto BestPlane = Estimator.GetBestModel();
	if (BestPlane == nullptr)
	{
		return false;
	}
	for (int i = 0; i < 3; i++)
	{
        center[i] = BestPlane->m_PointCenter[i];
	}
    for (int i = 0; i < 4; i++)
    {
        normal[i] = BestPlane->m_PlaneCoefs[i];
    }

	return true;
}

bool UtilSetpoint::MovingAvgFilter(Z2NormalAngle& z_to_normal, double angle_deg)
{
    const static int BUF_SIZE = 30;
    // Initialize
    if (!z_to_normal.is_init){
        for (int i = 0; i < BUF_SIZE; i++){
            z_to_normal.angle_deg_buf.push_back(angle_deg);
        }

        z_to_normal.is_init = true;
    }

    // Shift
    for (int i = 0; i < BUF_SIZE - 1; i++){
        z_to_normal.angle_deg_buf.at(i) = z_to_normal.angle_deg_buf.at(i+1);
    }
    z_to_normal.angle_deg_buf.at(BUF_SIZE - 1) = angle_deg;

    // Summation
    double sum = 0;
    for (auto a : z_to_normal.angle_deg_buf){
        sum += a;
    }

    // Average
    z_to_normal.angle_deg = sum/(double)BUF_SIZE;
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