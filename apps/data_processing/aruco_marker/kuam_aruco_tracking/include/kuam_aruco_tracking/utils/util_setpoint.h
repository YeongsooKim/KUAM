#ifndef __ARUCO_TRACKING_UTIL_SETPOINT_H__
#define __ARUCO_TRACKING_UTIL_SETPOINT_H__

#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf/LinearMath/Quaternion.h> // tf::quaternion

using namespace std;

namespace kuam{
class UtilSetpoint
{
public:
    UtilSetpoint();
    ~UtilSetpoint();

private:
    const double DEG2RAD;
    const double RAD2DEG;

public:
    void Translate(geometry_msgs::Pose& pose, const int id, 
                            const float big_marker_trans, const float medium_marker_trans, const float small_marker_trans);
    float GetYawRad(const geometry_msgs::Quaternion& quat_msg);
    float GetYawDeg(const geometry_msgs::Quaternion& quat_msg);
    bool IsValid(geometry_msgs::Pose pose);
    geometry_msgs::Pose GetSetpoint(vector<geometry_msgs::Pose> poses);
};
}

#endif // __ARUCO_TRACKING_UTIL_SETPOINT_H__