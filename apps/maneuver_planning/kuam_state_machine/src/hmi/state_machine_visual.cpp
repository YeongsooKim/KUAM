#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/format.hpp>

#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include "tf2_ros/transform_listener.h" // tf::quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/ColorRGBA.h>
#include <kuam_state_machine/utils.h>
#include <kuam_msgs/Setpoint.h>
#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/Status.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/HomePosition.h>
#include <jsk_rviz_plugins/OverlayText.h>

using namespace std;

namespace kuam{

//// target markers
std_msgs::ColorRGBA RED;
std_msgs::ColorRGBA GREEN;
std_msgs::ColorRGBA BLUE;
std_msgs::ColorRGBA YELLOW;
std_msgs::ColorRGBA WHITE;
std_msgs::ColorRGBA PURPLE;
std_msgs::ColorRGBA CYAN;
std_msgs::ColorRGBA ORANGE;

enum class SETPOINT : int{
    GLOBAL,
    RELATIVE,

    ItemNum
};

struct MetaMarkers{
    visualization_msgs::MarkerArray self;
    visualization_msgs::Marker trajectory;
    visualization_msgs::Marker current_point;
    visualization_msgs::Marker txt;

    bool is_trajectory_add;
    bool is_current_point_add;
    bool is_txt_add;
};

struct TextDatum{
    string coverage;
    string mode;
    string offb_state;
    string landing_state;
    string is_detected;
    string is_pass_landing_standby;
    string tasklist;
    string ego_height_m;
    string home_altitude_m;
    string setpoint_local_h_m;
    string ego_global_alt_m;
    string offset_alt_m;
    string battery_per;
};

class Visualizer
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;

    Utils m_utils;
    TextDatum m_text_datum;

public:
    Visualizer();
    ~Visualizer();

private:
    // Subscriber
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_state_sub;

    // Publisher
    ros::Publisher m_setpoints_pub;
    ros::Publisher m_text_pub;

    // Timer
    ros::Timer m_text_pub_cb_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    float m_text_width_param;
    float m_text_height_param;
    float m_text_left_param;
    float m_text_top_param;
    
    string m_data_ns_param;
    string m_maneuver_ns_param;
    float m_standby_dist_th_m_param;
    float m_landing_standby_alt_m_param;
    float m_small_marker_size_m_param;
    float m_medium_marker_size_m_param;
    float m_big_marker_size_m_param;
    vector<int> m_small_marker_ids_param;
    vector<int> m_medium_marker_ids_param;
    vector<int> m_big_marker_ids_param;

    // Marker
    using aruco = vector < MetaMarkers >;
    using id2aruco = map < int, aruco >;
    id2aruco m_targets_markers;
    MetaMarkers m_ego_markers; // target markers
    visualization_msgs::MarkerArray m_global_markers;
    visualization_msgs::MarkerArray m_gt_markers;
    vector<visualization_msgs::MarkerArray> m_setpoints_markers;

    geographic_msgs::GeoPoint m_home_position;
    vector < vector < geometry_msgs::Point > > m_setpoints;
    nav_msgs::Odometry m_ego_pose;
    vector<int> m_marker_ids;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;


private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr);
    void StateMachineStatusCallback(const kuam_msgs::Status::ConstPtr &status_ptr);
    void TextPubCallback(const ros::TimerEvent& event);
    void GTPubCallback(const ros::TimerEvent& event);

    string BoostColor(string str, std_msgs::ColorRGBA rgba);

};


Visualizer::Visualizer() :
    m_p_nh("~"),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[state_machine_visual] Fail GetParam %s", m_err_param.c_str());
    InitROS();

    // m_setpoints.resize((int)SETPOINT::ItemNum);
    InitMarkers();
}


Visualizer::~Visualizer()
{
}

bool Visualizer::InitFlag()
{

    return true;
}

bool Visualizer::GetParam()
{
    // if (!m_p_nh.getParam("data_ns", m_data_ns_param)) { m_err_param = "data_ns"; return false; }
    // if (!m_p_nh.getParam("state_machine/standby_dist_th_m", m_standby_dist_th_m_param)) { m_err_param = "standby_dist_th_m"; return false; }
    // if (!m_p_nh.getParam("state_machine/landing_standby_alt_m", m_landing_standby_alt_m_param)) { m_err_param = "landing_standby_alt_m"; return false; }
    if (!m_p_nh.getParam("width", m_text_width_param)) { m_err_param = "width"; return false; }
    if (!m_p_nh.getParam("height", m_text_height_param)) { m_err_param = "height"; return false; }
    if (!m_p_nh.getParam("left", m_text_left_param)) { m_err_param = "left"; return false; }
    if (!m_p_nh.getParam("top", m_text_top_param)) { m_err_param = "top"; return false; }    
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }

    return true;
}

bool Visualizer::InitROS()
{
    // Initialize subscriber
    m_setpoint_sub = 
        m_nh.subscribe<kuam_msgs::Setpoint>("state_machine/setpoint", 10, boost::bind(&Visualizer::SetpointCallback, this, _1));
    m_state_sub =
        m_nh.subscribe<kuam_msgs::Status>("state_machine/status", 10, boost::bind(&Visualizer::StateMachineStatusCallback, this, _1));

    // Initialize publisher
    m_setpoints_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("setpoint_markerarray", 10);
    m_text_pub = m_p_nh.advertise<jsk_rviz_plugins::OverlayText>("text", 10);
    
    // Initialize timer
    m_text_pub_cb_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Visualizer::TextPubCallback, this);

    return true;
}

bool Visualizer::InitMarkers()
{
    RED.r = 1.0f;       RED.g = 0.0f;       RED.b = 0.0f;
    GREEN.r = 0.0f;     GREEN.g = 1.0f;     GREEN.b = 0.0f;
    BLUE.r = 0.0f;      BLUE.g = 0.0f;      BLUE.b = 1.0f;
    YELLOW.r = 1.0f;    YELLOW.g = 1.0f;    YELLOW.b = 0.0f;
    WHITE.r = 1.0f;     WHITE.g = 1.0f;     WHITE.b = 1.0f;
    PURPLE.r = 1.0f;    PURPLE.g = 0.0f;    PURPLE.b = 1.0f;
    CYAN.r = 0.0f;      CYAN.g = 1.0f;      CYAN.b = 1.0f;
    ORANGE.r = 255.0/255.0; ORANGE.g = 102.0/255.0; ORANGE.b = 0.0;

    //// setpoint
    // m_setpoints_markers.resize((int)SETPOINT::ItemNum);
    m_text_datum.mode = "\n";
    m_text_datum.offb_state = "\n";
    m_text_datum.coverage = "\n";
    m_text_datum.is_detected = "\n";
}

void Visualizer::SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr)
{
    kuam_msgs::Setpoint setpoint = *setpoint_ptr; 

    // Set text marker
    m_text_datum.home_altitude_m = to_string(setpoint.home_altitude_m);
    m_text_datum.setpoint_local_h_m = to_string(setpoint.local_height_m);

    string coverage;
    if (setpoint.is_global) coverage = "GPS coverage";
    else coverage = "Camera coverage";

    m_text_datum.coverage = "Coverage: " + coverage + "\n";

    // if (m_text_datum.cur_state == "LANDING\n"){
    //     if (!setpoint.landing_state.is_pass_landing_standby){
    //         m_text_datum.landing_state = "Standby\n";
    //     }
    //     else {
    //         if (!setpoint.landing_state.is_land){
    //             m_text_datum.landing_state = "KUAM ctrl\n";
    //         }
    //         else {
    //             m_text_datum.landing_state = "FC ctrl\n";
    //         }
    //     }
    // }
    // else {
    //     m_text_datum.landing_state = "Not in landing\n";
    // }

    string is_detected;
    if (setpoint.landing_state.is_detected) is_detected = BoostColor("TRUE", GREEN);
    else is_detected = is_detected = BoostColor("FALSE", RED);

    m_text_datum.is_detected = "Is detected: " + is_detected + "\n";

    // string is_pass_landing_standby;
    // if (setpoint.landing_state.is_pass_landing_standby){
    //     is_pass_landing_standby = "TRUE\n";
    //     m_text_datum.is_pass_landing_standby = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
    //          % is_pass_landing_standby % 0.0 % 255.0 % 0.0 % 1.0).str();
    // }
    // else{
    //     is_pass_landing_standby = "FALSE\n";
    //     m_text_datum.is_pass_landing_standby = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
    //          % is_pass_landing_standby % 255.0 % 0.0 % 0.0 % 1.0).str();
    // }
    
    // kuam_msgs::Setpoint setpoint = *setpoint_ptr; 

    // // Set setpoint marker
    // static bool prev_coord = !setpoint.is_global;
    // if (setpoint.is_global){
    //     static geometry_msgs::Point prev_global_point;
    //     static unsigned int g_cnt = 0;
    //     bool is_init = false;
    //     if (prev_coord != setpoint.is_global){
    //         prev_coord = setpoint.is_global;
    //         is_init = true;
    //         g_cnt++;

    //         //// Setpoints
    //         visualization_msgs::Marker sp_marker;
    //         std_msgs::ColorRGBA color = GREEN; color.a = 0.4f;

    //         sp_marker.header.frame_id = "map";
    //         sp_marker.ns = "setpoint/global";
    //         sp_marker.id = g_cnt;
    //         sp_marker.type = visualization_msgs::Marker::LINE_STRIP;
    //         sp_marker.action = visualization_msgs::Marker::ADD;
    //         sp_marker.scale.x = 0.05;
    //         sp_marker.pose.orientation.w = 1.0;
    //         sp_marker.pose.orientation.x = 0.0;
    //         sp_marker.pose.orientation.y = 0.0;
    //         sp_marker.pose.orientation.z = 0.0;
    //         sp_marker.color = color;
    //         sp_marker.lifetime = ros::Duration();

    //         m_setpoints_markers[(int)SETPOINT::GLOBAL].markers.push_back(sp_marker);
    //     }

    //     auto geopos = setpoint.geopose.position;
    //     auto lat = geopos.latitude;
    //     auto lon = geopos.longitude;
    //     auto alt = setpoint.local_height_m;
    //     auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
        
    //     float dist = m_utils.Distance3D(p.position, prev_global_point);
    //     if ((dist > 0.02) || is_init){
    //         m_setpoints_markers[(int)SETPOINT::GLOBAL].markers[g_cnt-1].points.push_back(p.position);
    //         prev_global_point = p.position;
    //     }
    // }
    // else {
    //     static geometry_msgs::Point prev_pos;
    //     static unsigned int r_cnt = 0;
    //     bool is_init = false;
    //     if (prev_coord != setpoint.is_global){
    //         prev_coord = setpoint.is_global;
    //         is_init = true;
    //     }

    //     float dist = m_utils.Distance3D(m_ego_markers.current_point.pose.position, prev_pos);
    //     if ((dist > 0.02) || is_init){
    //         auto x = setpoint.vel.linear.x;
    //         auto y = setpoint.vel.linear.y;
    //         auto z = setpoint.vel.linear.z;
    //         auto size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));

    //         visualization_msgs::Marker sp_marker;
    //         std_msgs::ColorRGBA color = PURPLE; color.a = 0.4f;
    //         sp_marker.header.frame_id = "map";
    //         sp_marker.header.stamp = ros::Time::now();
    //         sp_marker.ns = "setpoint/relative";
    //         sp_marker.id = r_cnt;
    //         sp_marker.type = visualization_msgs::Marker::ARROW;
    //         sp_marker.action = visualization_msgs::Marker::ADD;
    //         sp_marker.scale.x = size;
    //         sp_marker.scale.y = 0.05;
    //         sp_marker.scale.z = 0.05;
    //         sp_marker.pose.position = m_ego_markers.current_point.pose.position;
    //         sp_marker.pose.orientation = setpoint.pose.orientation;
    //         sp_marker.color = color;
    //         sp_marker.lifetime = ros::Duration();
    //         m_setpoints_markers[(int)SETPOINT::RELATIVE].markers.push_back(sp_marker);

    //         prev_pos = m_ego_markers.current_point.pose.position;
    //         r_cnt++;
    //     }
    // }
    
    // visualization_msgs::MarkerArray visualization_markers;
    // for (int coord = (int)SETPOINT::GLOBAL; coord < (int)SETPOINT::ItemNum; coord++){
    //     visualization_markers.markers.insert(visualization_markers.markers.end(), 
    //                                         m_setpoints_markers[coord].markers.begin(), m_setpoints_markers[coord].markers.end());
    // }

    // if (!setpoint.is_global){
    //     geometry_msgs::Pose p;
    //     geometry_msgs::TransformStamped transformStamped;
    //     geometry_msgs::Pose transformed_pose;
    //     try{
    //         transformStamped = 
    //             m_tfBuffer.lookupTransform("map", "landing_point", ros::Time(0));
    //         tf2::doTransform(p, transformed_pose, transformStamped);
    //     }
    //     catch (tf2::TransformException &ex) {
    //         ROS_WARN("%s", ex.what());
    //         ros::Duration(1.0).sleep();
    //     }

    //     visualization_msgs::Marker landing_point;
    //     landing_point.header.frame_id = "map";
    //     landing_point.header.stamp = ros::Time::now();
    //     landing_point.ns = "setpoint/landing_point";
    //     landing_point.id = 0;
    //     landing_point.type = visualization_msgs::Marker::SPHERE;
    //     landing_point.action = visualization_msgs::Marker::ADD;
    //     landing_point.scale.x = 0.15;
    //     landing_point.scale.y = 0.15;
    //     landing_point.scale.z = 0.15;
    //     landing_point.pose.position = transformed_pose.position;
    //     landing_point.pose.orientation.x = 0.0;
    //     landing_point.pose.orientation.y = 0.0;
    //     landing_point.pose.orientation.z = 0.0;
    //     landing_point.pose.orientation.w = 1.0;
    //     landing_point.color = ORANGE;
    //     landing_point.color.a = 1.0f;
    //     landing_point.lifetime = ros::Duration(0.1);
    //     visualization_markers.markers.push_back(landing_point);
    // }
    // m_setpoints_pub.publish(visualization_markers);
}

void Visualizer::StateMachineStatusCallback(const kuam_msgs::Status::ConstPtr &status_ptr)
{
    string mode;
    if (status_ptr->mode == "EMERGY") mode = BoostColor(status_ptr->mode, RED);
    else mode = status_ptr->mode;

    m_text_datum.mode = "Mode: " + mode + "\n";
    m_text_datum.offb_state = "Offboard state: " + status_ptr->offb_state + "\n";
}

void Visualizer::TextPubCallback(const ros::TimerEvent& event)
{
    jsk_rviz_plugins::OverlayText text;
    std_msgs::ColorRGBA fg_color;   fg_color.r = 25.0/255.0;    fg_color.g = 1.0;           fg_color.b = 240.0/255.0;   fg_color.a = 1.0;
    std_msgs::ColorRGBA bg_color;   bg_color.r = 136.0/255.0;   bg_color.g = 138.0/255.0;   bg_color.b = 133.0/255.0;   bg_color.a = 0.35;

    text.width = m_text_width_param;
    text.height = m_text_height_param;
    text.left = m_text_left_param;
    text.top = m_text_top_param;
    text.text_size = 12;
    text.line_width = 2;
    text.font = "DejaVu Sans Mono";

    // auto is_pass_landing_standby = m_text_datum.is_pass_landing_standby;
    // auto ego_local_height_m = m_text_datum.ego_height_m;
    // auto landing_state = m_text_datum.landing_state;
    // auto tasklist = m_text_datum.tasklist;
    // auto home_altitude_m =  m_text_datum.home_altitude_m;
    // auto setpoint_local_h_m =  m_text_datum.setpoint_local_h_m;
    // auto offset_alt_m =  m_text_datum.offset_alt_m;
    // auto ego_global_alt_m = m_text_datum.ego_global_alt_m;
    // auto battery_per = m_text_datum.battery_per;
    
    text.text = 
        m_text_datum.mode + m_text_datum.offb_state +
        "\n" +
        m_text_datum.coverage + m_text_datum.is_detected;
        
        // "Coverage: " + coverage + "Is detected: " + is_detected + "Is pass landing standby: " + is_pass_landing_standby +
        // "\nLocal altitude: " + ego_local_height_m + 
        // "\nCurrent mode: " + mode + "Current state: " + 
        // offb_state + "\nLanding state: " + 
        // landing_state + "\nTask list: \n" + tasklist +
        // "\n---\n\nGlobal altitude: " + ego_global_alt_m + "[m]\nHome altitude: " + home_altitude_m + "[m]\nLocal setpoint alt: " + setpoint_local_h_m + "[m]\nOffset altitude: " + offset_alt_m + "[m]\n" +
        // "\nBattery percentage: " + battery_per;

    // text.text = 
        // "Coverage: " + coverage + "Is detected: " + is_detected + "Is pass landing standby: " + is_pass_landing_standby +
        // "\nLocal altitude: " + ego_local_height_m + 
        // "\nCurrent mode: " + mode + "Current state: " + 
        // offb_state + "\nLanding state: " + 
        // landing_state + "\nTask list: \n" + tasklist +
        // "\n---\n\nGlobal altitude: " + ego_global_alt_m + "[m]\nHome altitude: " + home_altitude_m + "[m]\nLocal setpoint alt: " + setpoint_local_h_m + "[m]\nOffset altitude: " + offset_alt_m + "[m]\n" +
        // "\nBattery percentage: " + battery_per;

    text.fg_color = fg_color;
    text.bg_color = bg_color;

    m_text_pub.publish(text);
}

string Visualizer::BoostColor(string str, std_msgs::ColorRGBA rgba)
{
    rgba.r *= 255.0; rgba.g *= 255.0; rgba.b *= 255.0; rgba.a = 1.0;
    auto color = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % str % rgba.r % rgba.g % rgba.b % rgba.a).str();
    return color;
}
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "state_machine_visual");
    kuam::Visualizer kuam_visualizer;

    ros::spin();

    return 0;
}
