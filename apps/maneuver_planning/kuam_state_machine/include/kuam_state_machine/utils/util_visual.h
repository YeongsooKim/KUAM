#ifndef __STATE_MACHINE_UTILS_VISUAL_H__
#define __STATE_MACHINE_UTILS_VISUAL_H__
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPoint.h>

#include "tf2_ros/transform_listener.h"
#include <tf/LinearMath/Quaternion.h> // tf::quaternion

namespace kuam{

struct Euler {
    double r;
    double p;
    double y;
};

class UtilVisual
{
public:
    UtilVisual();
    ~UtilVisual();

private:
    // const variable
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;


public:
    float Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    float Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    // ref http://www.movable-type.co.uk/scripts/latlong.html
    double DistanceFromLatLonInKm(geographic_msgs::GeoPoint point1, geographic_msgs::GeoPoint point2);
    double Deg2Rad(double degree);
    double Rad2Deg(double rad);
    geometry_msgs::Quaternion GetOrientation(geometry_msgs::Point src_pos, geometry_msgs::Point trg_pos);
    geometry_msgs::Pose ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);
    double Size(double x, double y);
    double Size(double x, double y, double z);
};
}

#endif // __STATE_MACHINE_UTILS_VISUAL_H__