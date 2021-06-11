#ifndef __UTILS_H__
#define __UTILS_H__
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>
#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include "tf2_ros/transform_listener.h" // tf::quaternion

namespace kuam{
class Utils
{
private:
    // const variable
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;

public:
    Utils();
    ~Utils();

public:
    float Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    float Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    double Deg2Rad(double degree);
    double Rad2Deg(double rad);
    double Size(double x, double y);
    double Size(double x, double y, double z);
    std::string ToString(double value);
    geometry_msgs::Quaternion GetOrientation(geometry_msgs::Point src_pos, geometry_msgs::Point trg_pos);
    geometry_msgs::Point ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);
};
}

#endif // __UTILS_H__