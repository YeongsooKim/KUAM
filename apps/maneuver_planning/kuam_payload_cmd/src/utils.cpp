#include "kuam_payload_cmd/utils.h"
#include "math.h"
#include <iomanip>

namespace kuam
{

Utils::Utils() :
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371.0)
{}

Utils::~Utils() {}

Euler Utils::Quat2Euler(const geometry_msgs::Quaternion& quat_msg)
{
	tf2::Quaternion quat_tf(
        quat_msg.x,
        quat_msg.y,
        quat_msg.z,
        quat_msg.w);
	double roll, pitch, yaw;
	tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

	Euler euler = {roll, pitch, yaw};

	return euler;
}
}