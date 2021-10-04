#include "kuam_state_machine/utils/util_visual.h"
#include "math.h"
#include <iomanip>
#include <sstream>
#include <cmath>
namespace kuam
{

UtilVisual::UtilVisual() :
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371.0)
{}

UtilVisual::~UtilVisual() {}

float UtilVisual::Distance2D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto distance_m = Size(delta_x, delta_y);

    return distance_m;
}

float UtilVisual::Distance3D(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    auto delta_x = point1.x - point2.x;
    auto delta_y = point1.y - point2.y;
    auto delta_z = point1.z - point2.z;
    auto distance_m = Size(delta_x, delta_y, delta_z);

    return distance_m;
}

// ref http://www.movable-type.co.uk/scripts/latlong.html
double UtilVisual::DistanceFromLatLonInKm(geographic_msgs::GeoPoint point1, geographic_msgs::GeoPoint point2)
{
    double lat1 = point1.latitude;
    double lon1 = point1.longitude;

    double lat2 = point2.latitude;
    double lon2 = point2.longitude;
    
    double dLat = Deg2Rad(lat2 - lat1);
    double dLon = Deg2Rad(lon2 - lon1);

    double a = 
        sin(dLat/2) * sin(dLat/2) + 
        cos(Deg2Rad(lat1)) * cos(Deg2Rad(lat2)) * 
        sin(dLon/2) * sin(dLon/2);

    double c = 2*atan2(sqrt(a), sqrt(1 - a)); 
    double distance_m = EARTH_RADIUS_M * c; // Distance in km
    return distance_m;
}

double UtilVisual::Deg2Rad(double degree)
{
    double rad = degree * (M_PI / 180.0);
	return rad; 
}

double UtilVisual::Rad2Deg(double rad)
{
	double degree = rad * (180.0 / M_PI);
	return degree;
}

geometry_msgs::Quaternion UtilVisual::GetOrientation(geometry_msgs::Point src_pos, geometry_msgs::Point trg_pos)
{
    auto dx = trg_pos.x - src_pos.x;
    auto dy = trg_pos.y - src_pos.y;
    auto yaw_rad = atan2(dy, dx);

    tf2::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, yaw_rad);
    geometry_msgs::Quaternion q_msg;
    q_msg.x = q_tf.x();
    q_msg.y = q_tf.y();
    q_msg.z = q_tf.z();
    q_msg.w = q_tf.w();

    return q_msg;
}


geometry_msgs::Pose UtilVisual::ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    dKappaLat = FnKappaLat( home_position.latitude , hgt );
    dKappaLon = FnKappaLon( home_position.latitude , hgt );

    geometry_msgs::Pose pose;

    pose.position.x = (lon-home_position.longitude)/dKappaLon;
    pose.position.y = (lat-home_position.latitude)/dKappaLat;
    pose.position.z = hgt;

    return pose;
}

double UtilVisual::FnKappaLat(double dRef_Latitude, double dHeight)
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

double UtilVisual::FnKappaLon(double dRef_Latitude, double dHeight)
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

double UtilVisual::Size(double x, double y)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0));
    return size;
}

double UtilVisual::Size(double x, double y, double z)
{
    double size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
    return size;
}
}