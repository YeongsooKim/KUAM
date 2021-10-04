#ifndef __ARUCO_TRACKING_UTIL_GEOMETRY_H__
#define __ARUCO_TRACKING_UTIL_GEOMETRY_H__

#include <geometry_msgs/Point.h>

namespace kuam{
class UtilGeometry
{
public:
    UtilGeometry();
    ~UtilGeometry();

public:
    float Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    float Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    double Size(double x, double y);
    double Size(double x, double y, double z);
    std::string ToString(double value);
};
}

#endif // __ARUCO_TRACKING_UTIL_GEOMETRY_H__