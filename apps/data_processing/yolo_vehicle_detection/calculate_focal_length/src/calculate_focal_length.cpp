#include <ros/ros.h>
#include <string>
#include <vector>
#include <cmath>

// Messages
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <geographic_msgs/GeoPoint.h>
#include <mavros_msgs/HomePosition.h>
#include <sensor_msgs/NavSatFix.h>

// Tf

using namespace std;

namespace kuam{

struct Center{
    double x;
    double y;
};

class CalFocalLength
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    ros::NodeHandle m_p_nh;

public:
    CalFocalLength();
    ~CalFocalLength();

private:
    // Subscriber
    ros::Subscriber m_bounding_box_sub;
    ros::Subscriber m_local_pose_sub;
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_global_ego_pos_sub;

    // Publisher
    // ros::Publisher m_vehicle_state_pub;

    // Timer
    ros::Timer m_process_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    float m_image_plane_width_m_param;
    float m_image_plane_height_m_param;

    // Const value
    const double GEOD_A;
    const double GEOD_e2;
    const double EARTH_RADIUS_M;

    // tf

    // Variable
    geometry_msgs::Point m_ego_point;
    double m_vehicle_dist;
    geographic_msgs::GeoPoint m_home_position;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();

    // void ProcessTimerCallback(const ros::TimerEvent& event);
    void BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr bounding_box_ptr);
    inline void EgoLocalCallback(const nav_msgs::Odometry::ConstPtr local_ptr) { m_ego_point = local_ptr->pose.pose.position; }
    inline void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr) { m_home_position = home_ptr->geo; }
    void EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr);
    
    geometry_msgs::Pose ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position);
    double FnKappaLat(double dRef_Latitude, double dHeight);
    double FnKappaLon(double dRef_Latitude, double dHeight);
};

CalFocalLength::CalFocalLength() :
    m_p_nh("~"),
    GEOD_A(6378137.0),
    GEOD_e2(0.00669437999014),
    EARTH_RADIUS_M(6371.0)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_tracking] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

CalFocalLength::~CalFocalLength()
{
}

bool CalFocalLength::InitFlag()
{
    return true;
}

bool CalFocalLength::GetParam()
{
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("image_plane_width_m", m_image_plane_width_m_param)) { m_err_param = "image_plane_width_m"; return false; }
    if (!m_p_nh.getParam("image_plane_height_m", m_image_plane_height_m_param)) { m_err_param = "image_plane_height_m"; return false; }

    return true;
}

bool CalFocalLength::InitROS()
{
    // Initialize subscriber
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&CalFocalLength::HomePositionCallback, this, _1));
    m_global_ego_pos_sub =
        m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&CalFocalLength::EgoGlobalPosCallback, this, _1));
    m_bounding_box_sub = m_nh.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, boost::bind(&CalFocalLength::BoundingBoxCallback, this, _1));
    m_local_pose_sub = m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&CalFocalLength::EgoLocalCallback, this, _1));
    
    // Initialize publisher
    // m_vehicle_state_pub = m_p_nh.advertise<kuam_msgs::VehicleState>("vehicle_state", 10);
    
    // Initialize timer
    // m_process_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &CalFocalLength::ProcessTimerCallback, this);

    return true;
}

void CalFocalLength::BoundingBoxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr bounding_box_ptr)
{
    if (bounding_box_ptr->bounding_boxes.empty())
        return;

    // Calculate center point of bounding box
    vector<Center> bounding_boxes;
    bounding_boxes.clear();
    for (auto bb : bounding_box_ptr->bounding_boxes){
        auto dx = bb.xmax - bb.xmin;
        auto dy = bb.ymax - bb.ymin;

        Center c;
        c.x = dx/2.0 + bb.xmin;
        c.y = dy/2.0 + bb.ymin;
        bounding_boxes.push_back(c);
    }

    // Normalized image plane coordinate (optical axis is origin)
    vector<Center> norm_bounding_boxes;
    for (auto bb : bounding_boxes){
        Center c;
        c.x = bb.x - m_image_plane_width_m_param/2.0;
        c.y = -bb.y + m_image_plane_height_m_param/2.0;

        norm_bounding_boxes.push_back(c);
    }

    // Calculate relative position relation to camera origin in camera_link
    double sum_x = 0.0; double sum_y = 0.0;
    for (auto bb : norm_bounding_boxes){
        sum_x += bb.x;
        sum_y += bb.y;
    }

    geometry_msgs::Point vehicle;
    vehicle.x = sum_x / norm_bounding_boxes.size();
    vehicle.y = -sum_y / norm_bounding_boxes.size();
    double pixel_dist = sqrt(pow(vehicle.x, 2.0) + pow(vehicle.y, 2.0));
    double height = m_ego_point.z + 1.925;
    double focal_length = pixel_dist*height/m_vehicle_dist;

    ROS_WARN("focal length: %f, height: %f, pixel dist: %f, vehicle dist: %f", focal_length, height, pixel_dist, m_vehicle_dist);
}

void CalFocalLength::EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr)
{
    // Set ego markers
    auto lat = ego_ptr->latitude;
    auto lon = ego_ptr->longitude;
    auto alt = m_ego_point.z;
    auto p = ConvertToMapFrame(lat, lon, alt, m_home_position);

    m_vehicle_dist = sqrt(pow(p.position.x, 2.0) + pow(p.position.y, 2.0));
}


geometry_msgs::Pose CalFocalLength::ConvertToMapFrame(double lat, double lon, double hgt, geographic_msgs::GeoPoint home_position)
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

double CalFocalLength::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(dRef_Latitude*M_PI/180.0), 2));
	dM = GEOD_A * (1 - GEOD_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = (1 / (dM + dHeight))*180.0/M_PI;

	return dKappaLat;
}

double CalFocalLength::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - GEOD_e2 * pow(sin(dRef_Latitude*M_PI/180.0), 2));
	dN = GEOD_A / Denominator;

	// Curvature for the meridian
	dKappaLon = (1 / ((dN + dHeight) * cos(dRef_Latitude*M_PI/180.0)))*180.0/M_PI;

	return dKappaLon;
}
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "calculate_focal_length");
    kuam::CalFocalLength calculate_focal_length;

    ros::spin();

    return 0;
} 