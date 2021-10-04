#ifndef __GLOBAL_UTILS_H__
#define __GLOBAL_UTILS_H__
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geographic_msgs/GeoPoint.h>

#include "tf2_ros/transform_listener.h"
#include <tf/LinearMath/Quaternion.h> // tf::quaternion

namespace kuam{
class Utils
{
private:
    // const variable
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;
    const double DEG2RAD;
    const double RAD2DEG;

public:
    Utils();
    ~Utils();

public:
    geometry_msgs::Pose ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);
};
}

#endif // __GLOBAL_UTILS_H__