#ifndef __UTILS_H__
#define __UTILS_H__
#include <geometry_msgs/Point.h>

namespace kuam{
class Utils
{
public:
    Utils();
    ~Utils();

public:
    float Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    float Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2);
    double Size(double x, double y);
    double Size(double x, double y, double z);
    std::string ToString(double value);
};
}

#endif // __UTILS_H__