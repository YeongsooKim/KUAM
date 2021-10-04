#include <kuam_aruco_tracking/utils/util_geometry.h>
#include <sstream>
#include <math.h>

namespace kuam
{

UtilGeometry::UtilGeometry()
{}

UtilGeometry::~UtilGeometry() {}

float UtilGeometry::Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto distance_m = Size(delta_x, delta_y);

    return distance_m;
}

float UtilGeometry::Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = Size(delta_x, delta_y, delta_z);

    return distance_m;
}

double UtilGeometry::Size(double x, double y)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0));
    return size;
}

double UtilGeometry::Size(double x, double y, double z)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
    return size;
}

std::string UtilGeometry::ToString(double value)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    std::string s = stream.str();

    return s;
}
}