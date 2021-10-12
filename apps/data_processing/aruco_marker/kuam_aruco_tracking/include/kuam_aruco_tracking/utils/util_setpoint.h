#ifndef __ARUCO_TRACKING_UTIL_SETPOINT_H__
#define __ARUCO_TRACKING_UTIL_SETPOINT_H__

#include <vector>
#include <iostream>
#include "GRANSAC.hpp"
#include "PlaneModel.hpp"
#include <omp.h>
#include <opencv2/opencv.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>

#include <tf2/LinearMath/Quaternion.h> // tf::quaternion
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

namespace kuam{

struct Z2NormalAngle{
    bool is_init;
    vector<double> angle_deg_buf;
    double angle_deg;
};

class UtilSetpoint
{
public:
    UtilSetpoint();
    ~UtilSetpoint();

private:
    const double DEG2RAD;
    const double RAD2DEG;

public:
    void Transform(geometry_msgs::Pose& pose, const int id, 
                            const float big_marker_trans, const float medium_marker_trans, const float small_marker_trans);
    geometry_msgs::Pose GetSetpoint(vector<geometry_msgs::Pose> poses);
    bool GeneratePlane(const vector<kuam_msgs::ArucoState>& markers, 
                            geometry_msgs::PoseStamped& plane, Z2NormalAngle& z_to_normal, 
                            const double& plane_threshold, const int& iterations, const double& angle_threshold, bool& is_valid);
private:
    float GetYawRad(const geometry_msgs::Quaternion& quat_msg);
    float GetYawDeg(const geometry_msgs::Quaternion& quat_msg);
    bool PlaneFitting(const std::vector<Vector3VP> &points_input, double* center, double* normal, double threshold, double iterations);
    bool MovingAvgFilter(Z2NormalAngle& z_to_normal, double angle_deg);
};
}

#endif // __ARUCO_TRACKING_UTIL_SETPOINT_H__