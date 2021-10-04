#ifndef __PAYLOAD_CMD_UTILS_H__
#define __PAYLOAD_CMD_UTILS_H__
#include <geometry_msgs/Quaternion.h>

#include "tf2_ros/transform_listener.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h" // tf::quaternion

namespace kuam{

struct Euler {
    double r;
    double p;
    double y;
};

class Utils
{
public:
    Utils();
    ~Utils();

private:
    // const variable
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;


public:
    Euler Quat2Euler(const geometry_msgs::Quaternion& quat_msg);
};
}

#endif // __PAYLOAD_CMD_UTILS_H__