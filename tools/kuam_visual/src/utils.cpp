#include <kuam_visual/utils.h>
#include <sstream>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>

namespace kuam
{

Utils::Utils() :
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371e3)
{}

Utils::~Utils() {}

float Utils::Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto distance_m = Size(delta_x, delta_y);

    return distance_m;
}

float Utils::Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = Size(delta_x, delta_y, delta_z);

    return distance_m;
}

double Utils::Deg2Rad(double degree)
{
    double rad = degree * (M_PI / 180.0);
	return rad; 
}

double Utils::Rad2Deg(double rad)
{
	double degree = rad * (180.0 / M_PI);
	return degree;
}

double Utils::Size(double x, double y)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0));
    return size;
}

double Utils::Size(double x, double y, double z)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
    return size;
}

std::string Utils::ToString(double value)
{
    std::stringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    std::string s = stream.str();

    return s;
}

geometry_msgs::Quaternion Utils::GetOrientation(geometry_msgs::Point src_pos, geometry_msgs::Point trg_pos)
{
    auto dx = trg_pos.x - src_pos.x;
    auto dy = trg_pos.y - src_pos.y;
    auto yaw_rad = atan2(dy, dx);

    tf2::Quaternion tf_q;
    tf_q.setRPY(0.0, 0.0, yaw_rad);

    geometry_msgs::Quaternion geometry_q;
    geometry_q.x = tf_q.x();
    geometry_q.y = tf_q.y();
    geometry_q.z = tf_q.z();
    geometry_q.w = tf_q.w();

    return geometry_q;
}

geometry_msgs::Point Utils::ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    dKappaLat = FnKappaLat( home_position.latitude , hgt );
    dKappaLon = FnKappaLon( home_position.latitude , hgt );

    geometry_msgs::Point p;

    p.x = (lon-home_position.longitude)/dKappaLon;
    p.y = (lat-home_position.latitude)/dKappaLat;
    p.z = hgt;

    return p;
}

double Utils::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(Deg2Rad(dRef_Latitude)), 2));
	dM = GEOD_A * (1 - GEOD_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = Rad2Deg(1 / (dM + dHeight));

	return dKappaLat;
}

double Utils::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(Deg2Rad(dRef_Latitude)), 2));
	dN = GEOD_A / Denominator;

	// Curvature for the meridian
	dKappaLon = Rad2Deg(1 / ((dN + dHeight) * cos(Deg2Rad(dRef_Latitude))));

	return dKappaLon;
}

}