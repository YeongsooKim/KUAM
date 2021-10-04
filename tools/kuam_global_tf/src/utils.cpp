#include "kuam_global_tf/utils.h"
#include <cmath>

namespace kuam
{

Utils::Utils() :
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371e3),
    DEG2RAD(M_PI/180.0),
    RAD2DEG(180.0/M_PI)
{}

Utils::~Utils() {}


geometry_msgs::Pose Utils::ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position)
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

double Utils::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(dRef_Latitude*DEG2RAD), 2));
	dM = GEOD_A * (1 - GEOD_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = RAD2DEG*(1 / (dM + dHeight));

	return dKappaLat;
}

double Utils::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(dRef_Latitude*DEG2RAD), 2));
	dN = GEOD_A / Denominator;

	// Curvature for the meridian
	dKappaLon = RAD2DEG*(1 / ((dN + dHeight) * cos(dRef_Latitude*DEG2RAD)));

	return dKappaLon;
}
}