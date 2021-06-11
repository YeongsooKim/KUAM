#include <ros/ros.h>
#include <string>

#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include "tf2_ros/transform_listener.h" // tf::quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <kuam_visual/utils.h>

#include <kuam_aruco_tracking/aruco_tracking.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/Waypoints.h>
#include <kuam_msgs/Setpoint.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <std_msgs/ColorRGBA.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <mavros_msgs/HomePosition.h>

using namespace std;
using namespace cv;

namespace kuam{

    //// target markers
std_msgs::ColorRGBA RED;
std_msgs::ColorRGBA GREEN;
std_msgs::ColorRGBA BLUE;
std_msgs::ColorRGBA YELLOW;
std_msgs::ColorRGBA WHITE;

struct MetaMarkers{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker trajectory;
    visualization_msgs::Marker current_point;
    visualization_msgs::Marker txt;

    bool is_trajectory_add;
    bool is_current_point_add;
    bool is_txt_add;
};

class KuamVisualizer
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    Utils m_utils;

public:
    KuamVisualizer();
    ~KuamVisualizer();

private:
    // // Subscriber
    ros::Subscriber m_aruco_visual_sub;
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_global_waypoints_sub;
    ros::Subscriber m_setpoint_sub;

    // // Publisher
    ros::Publisher m_aruco_pub;
    ros::Publisher m_global_path_pub;
    ros::Publisher m_setpoints_pub;

    // // Timer
    // ros::Timer m_image_timer;

    // // Param
    string m_data_ns_param;
    string m_maneuver_ns_param;
    bool m_is_home_set;
    bool m_is_init_global_path;

    vector<MetaMarkers> m_target_markers; // target markers
    visualization_msgs::MarkerArray m_global;
    visualization_msgs::Marker m_setpoints;
    geographic_msgs::GeoPoint m_home_position;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;


private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void ArucoVisualCallback(const kuam_msgs::ArucoVisual::ConstPtr &aruco_msg_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr);
    void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr);
 
};


KuamVisualizer::KuamVisualizer() :
    m_data_ns_param("missing"),
    m_maneuver_ns_param("missing"),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[kuam_visual] Fail GetParam");
    InitROS();
    InitMarkers();


}


KuamVisualizer::~KuamVisualizer()
{
}

bool KuamVisualizer::InitFlag()
{
    m_is_home_set = false;
    m_is_init_global_path = false;

    return true;
}

bool KuamVisualizer::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/data_ns", m_data_ns_param);
    m_nh.getParam(nd_name + "/maneuver_ns", m_maneuver_ns_param);

    if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("[kuam_visual] m_data_ns_param is missing"); return false; }
    else if (m_maneuver_ns_param == "missing") { ROS_ERROR_STREAM("[kuam_visual] m_maneuver_ns_param is missing"); return false; }
    // else if (__isnan(m_big_marker_id_param)) { ROS_ERROR_STREAM("[kuam_visual] m_big_marker_id_param is NAN"); return false; }

    return true;
}

bool KuamVisualizer::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();
    string ns_name = ros::this_node::getNamespace();

    // Initialize subscriber
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&KuamVisualizer::HomePositionCallback, this, _1));

    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }

    m_aruco_visual_sub = 
        m_nh.subscribe<kuam_msgs::ArucoVisual>(m_data_ns_param + "/aruco_tracking/aruco_visual", 1, boost::bind(&KuamVisualizer::ArucoVisualCallback, this, _1));
    m_global_waypoints_sub = 
        m_nh.subscribe<kuam_msgs::Waypoints>(m_data_ns_param + "/csv_parser/waypoints", 10, boost::bind(&KuamVisualizer::WaypointsCallback, this, _1));
    m_setpoint_sub = 
        m_nh.subscribe<kuam_msgs::Setpoint>(m_maneuver_ns_param + "/state_machine/setpoint", 10, boost::bind(&KuamVisualizer::SetpointCallback, this, _1));

    // Initialize publisher
    m_aruco_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/aruco_markerarray", 10);
    m_global_path_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/global_path_markerarray", 10);
    m_setpoints_pub = m_nh.advertise<visualization_msgs::Marker>(nd_name + "/setpoint_marker", 10);
    
    // Initialize timer
    // m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &KuamVisualizer::ProcessTimerCallback, this);

    return true;
}


bool KuamVisualizer::InitMarkers()
{
    RED.r = 1.0f;       RED.g = 0.0f;       RED.b = 0.0f;
    GREEN.r = 0.0f;     GREEN.g = 1.0f;     GREEN.b = 0.0f;
    BLUE.r = 0.0f;      BLUE.g = 0.0f;      BLUE.b = 1.0f;
    YELLOW.r = 1.0f;    YELLOW.g = 1.0f;    YELLOW.b = 0.0f;
    WHITE.r = 1.0f;     WHITE.g = 1.0f;     WHITE.b = 1.0f;

    //// Global path
    visualization_msgs::Marker path;
    path.header.frame_id = "map";
    path.ns = "global/path";
    path.id = 0;
    path.type = visualization_msgs::Marker::LINE_STRIP;
    path.action = visualization_msgs::Marker::ADD;
    path.scale.x = 0.05;
    path.pose.orientation.w = 1.0;
    path.pose.orientation.x = 0.0;
    path.pose.orientation.y = 0.0;
    path.pose.orientation.z = 0.0;
    path.color = RED;
    path.color.a = 0.4f;
    path.lifetime = ros::Duration();
    m_global.markers.push_back(path);

    //// Setpoints
    m_setpoints.header.frame_id = "map";
    m_setpoints.ns = "setpoints";
    m_setpoints.id = 0;
    m_setpoints.type = visualization_msgs::Marker::LINE_STRIP;
    m_setpoints.action = visualization_msgs::Marker::ADD;
    m_setpoints.scale.x = 0.05;
    m_setpoints.pose.orientation.w = 1.0;
    m_setpoints.pose.orientation.x = 0.0;
    m_setpoints.pose.orientation.y = 0.0;
    m_setpoints.pose.orientation.z = 0.0;
    m_setpoints.color = GREEN;
    m_setpoints.color.a = 0.4f;
    m_setpoints.lifetime = ros::Duration();

    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        MetaMarkers target_marker;
        // Without filter. GREEN
        target_marker.trajectory.ns = "target/trajectory";
        target_marker.trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        target_marker.trajectory.action = visualization_msgs::Marker::ADD;
        target_marker.trajectory.scale.x = 0.05;
        target_marker.trajectory.pose.orientation.w = 1.0;
        target_marker.trajectory.pose.orientation.x = 0.0;
        target_marker.trajectory.pose.orientation.y = 0.0;
        target_marker.trajectory.pose.orientation.z = 0.0;
        target_marker.trajectory.lifetime = ros::Duration(0.3);
        target_marker.is_trajectory_add = false;

        target_marker.current_point.ns = "target/current_point";
        target_marker.current_point.type = visualization_msgs::Marker::SPHERE;
        target_marker.current_point.action = visualization_msgs::Marker::ADD;
        target_marker.current_point.scale.x = 0.15;
        target_marker.current_point.scale.y = 0.15;
        target_marker.current_point.scale.z = 0.15;
        target_marker.current_point.pose.orientation.w = 1.0;
        target_marker.current_point.pose.orientation.x = 0.0;
        target_marker.current_point.pose.orientation.y = 0.0;
        target_marker.current_point.pose.orientation.z = 0.0;
        target_marker.current_point.lifetime = ros::Duration(0.3);
        target_marker.is_current_point_add = false;
        
        target_marker.txt.ns = "target/txt";
        target_marker.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        target_marker.txt.action = visualization_msgs::Marker::ADD;
        target_marker.txt.scale.z = 0.1;
        target_marker.txt.pose.orientation.w = 1.0;
        target_marker.txt.pose.orientation.x = 0.0;
        target_marker.txt.pose.orientation.y = 0.0;
        target_marker.txt.pose.orientation.z = 0.0;
        target_marker.txt.lifetime = ros::Duration(0.3);
        target_marker.is_txt_add = false;

        switch (method){
            case (int)EstimatingMethod::WOF:
                GREEN.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::WOF;
                target_marker.trajectory.color = GREEN;

                GREEN.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::WOF;
                target_marker.current_point.color = GREEN;
                
                WHITE.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::WOF;
                target_marker.txt.color = WHITE;

                break;
            case (int)EstimatingMethod::MAF:
                RED.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::MAF;
                target_marker.trajectory.color = RED;

                RED.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::MAF;
                target_marker.current_point.color = RED;
                
                WHITE.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::MAF;
                target_marker.txt.color = WHITE;

                break;
            case (int)EstimatingMethod::EMAF:
                YELLOW.a = 0.4f;
                target_marker.trajectory.id = (int)EstimatingMethod::EMAF;
                target_marker.trajectory.color = YELLOW;

                YELLOW.a = 1.0f;
                target_marker.current_point.id = (int)EstimatingMethod::EMAF;
                target_marker.current_point.color = YELLOW;
                
                WHITE.a = 1.0f;
                target_marker.txt.id = (int)EstimatingMethod::EMAF;
                target_marker.txt.color = WHITE;

                break;
            default:
                break;
        }
        m_target_markers.push_back(target_marker);
    }
}

void KuamVisualizer::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[kuam_visual] Home set");
    }
}

void KuamVisualizer::ArucoVisualCallback(const kuam_msgs::ArucoVisual::ConstPtr &aruco_msg_ptr)
{
    geometry_msgs::Point pos;
    geometry_msgs::Point origin;

    if (aruco_msg_ptr->is_detected){
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            if (aruco_msg_ptr->is_compare_mode || (method == aruco_msg_ptr->esti_method)){
                // current_point
                pos = aruco_msg_ptr->poses[method].position;

                m_target_markers[method].current_point.header.frame_id = aruco_msg_ptr->header.frame_id;
                m_target_markers[method].current_point.header.stamp = ros::Time(0);

                m_target_markers[method].current_point.pose.position = pos;
                m_target_markers[method].is_current_point_add = true;

                // txt
                m_target_markers[method].txt.header.frame_id = aruco_msg_ptr->header.frame_id;;
                m_target_markers[method].txt.header.stamp = ros::Time(0);

                m_target_markers[method].txt.pose.position = pos;
                m_target_markers[method].txt.pose.position.z += 0.2;
                m_target_markers[method].txt.text = "dist: " + m_utils.ToString(m_utils.Distance3D(origin, pos)) + " [m]";
                m_target_markers[method].is_txt_add = true;
            }
        }
    }

    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        m_target_markers[method].self.markers.clear();
        if (m_target_markers[method].is_trajectory_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].trajectory);
        if (m_target_markers[method].is_current_point_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].current_point);
        if (m_target_markers[method].is_txt_add) m_target_markers[method].self.markers.push_back(m_target_markers[method].txt);
        m_target_markers[method].is_trajectory_add = m_target_markers[method].is_current_point_add = m_target_markers[method].is_txt_add = false;
    }
    
    visualization_msgs::MarkerArray visualization_markers;
    for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
        visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                            m_target_markers[method].self.markers.begin(), m_target_markers[method].self.markers.end());
    }

    m_aruco_pub.publish(visualization_markers);
}

void KuamVisualizer::WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr)
{
    if (!m_is_init_global_path){
        m_is_init_global_path = true;

        // Get pose
        vector<geometry_msgs::Point> points;
        for (auto wp : wps_ptr->waypoints){
            auto lat = wp.geopose.position.latitude;
            auto lon = wp.geopose.position.longitude;
            auto alt = wp.geopose.position.altitude;
            auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
            points.push_back(p);
        }

        m_global.markers[0].header.stamp = ros::Time::now();
        m_global.markers[0].points = points;

        // Allocate orientation
        vector<geometry_msgs::Pose> poses;
        for (int i = 0; i < points.size(); i++){
            geometry_msgs::Pose p;
            if (points.size() != 1){
                if (i + 1 < points.size()){
                    auto orientation = m_utils.GetOrientation(points[i], points[i+1]);
                    p.orientation = orientation;
                    p.position = points[i];
                }
                else {
                    p.orientation = poses[i-1].orientation;
                    p.position = poses[i-1].position;
                }
                poses.push_back(p);
            }
            else{
                p.orientation.x = 0.0;
                p.orientation.y = 0.0;
                p.orientation.z = 0.0;
                p.orientation.w = 1.0;
                p.position = points[i];
                poses.push_back(p);
            }
        }

        for (int i = 0; i < poses.size(); i++){

            visualization_msgs::Marker yaw;
            yaw.header.frame_id = "map";
            yaw.header.stamp = ros::Time::now();
            yaw.ns = "global/yaw";
            yaw.id = i;
            yaw.type = visualization_msgs::Marker::ARROW;
            yaw.action = visualization_msgs::Marker::ADD;
            yaw.scale.x = 0.3;
            yaw.scale.y = 0.1;
            yaw.scale.z = 0.1;
            yaw.pose = poses[i];
            yaw.color = RED;
            yaw.color.a = 0.4f;
            yaw.lifetime = ros::Duration();
            m_global.markers.push_back(yaw);
        }
        m_global_path_pub.publish(m_global);
    }
}

void KuamVisualizer::SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr)
{
    kuam_msgs::Setpoint setpoint = *setpoint_ptr; 

    if (setpoint.is_global){
        auto geopos = setpoint.geopose.position;
        auto lat = geopos.latitude;
        auto lon = geopos.longitude;
        auto alt = setpoint.height;
        auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);

        m_setpoints.header.frame_id = "map";
        m_setpoints.header.stamp = ros::Time::now();
        m_setpoints.points.push_back(p);
    }
    else {
        if (m_setpoints.header.frame_id != setpoint.header.frame_id){

            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose transformed_pose;
            try{
                transformStamped = m_tfBuffer.lookupTransform(m_setpoints.header.frame_id, setpoint.header.frame_id, ros::Time(0));
                tf2::doTransform(setpoint.pose, transformed_pose, transformStamped);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            m_setpoints.points.push_back(transformed_pose.position);
        }
        else{
            m_setpoints.points.push_back(setpoint.pose.position);
        }
    }
    m_setpoints_pub.publish(m_setpoints);
}
}


int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "kuam_visual");
    kuam::KuamVisualizer kuam_visualizer;

    ros::spin();

    return 0;
}