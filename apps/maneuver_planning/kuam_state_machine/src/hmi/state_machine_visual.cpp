#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/format.hpp>

#include <tf2_ros/transform_listener.h> // tf::quaternion

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Float32.h>
#include <kuam_msgs/Setpoint.h>
#include <kuam_msgs/Status.h>
#include <geometry_msgs/Point.h>
#include <geographic_msgs/GeoPoint.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/HomePosition.h>

#include <jsk_rviz_plugins/OverlayText.h>
#include "kuam_state_machine/utils/util_visual.h"

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
std_msgs::ColorRGBA WINE;

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
    string is_marker_detected;
    string is_vehicle_detected;
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

    UtilVisual m_util_visual;
    TextDatum m_text_datum;

public:
    Visualizer();
    ~Visualizer();

private:
    // Subscriber
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_global_ego_pos_sub;
    ros::Subscriber m_local_ego_pose_sub;

    // Publisher
    ros::Publisher m_setpoints_pub;
    ros::Publisher m_text_pub;
    ros::Publisher m_ego_pub;
    ros::Publisher m_target_height_pub;

    // Timer
    ros::Timer m_text_pub_cb_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    float m_text_width_param;
    float m_text_height_param;
    float m_text_left_param;
    float m_text_top_param;

    // Flag
    bool m_is_home_set;
    
    // Marker
    MetaMarkers m_ego_markers; // target markers
    vector<visualization_msgs::MarkerArray> m_setpoints_markers;

    geographic_msgs::GeoPoint m_home_position;
    nav_msgs::Odometry m_ego_pose;
    float m_setpoint_home_altitude_m;
    float m_setpoint_local_height_m;

	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;


private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr);
    void EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr);
    inline void EgoLocalPoseCallback(const nav_msgs::Odometry::ConstPtr &ego_ptr) { m_ego_pose = *ego_ptr; }
    void StateMachineStatusCallback(const kuam_msgs::Status::ConstPtr &status_ptr);
    void TextPubCallback(const ros::TimerEvent& event);

    string BoostColor(string str, std_msgs::ColorRGBA rgba);
};


Visualizer::Visualizer() :
    m_p_nh("~"),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[state_machine_visual] Fail GetParam %s", m_err_param.c_str());
    InitROS();

    InitMarkers();
}


Visualizer::~Visualizer()
{
}

bool Visualizer::InitFlag()
{
    m_is_home_set = false;

    return true;
}

bool Visualizer::GetParam()
{
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
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&Visualizer::HomePositionCallback, this, _1));
    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }
    m_setpoint_sub = 
        m_nh.subscribe<kuam_msgs::Setpoint>("state_machine/setpoint", 10, boost::bind(&Visualizer::SetpointCallback, this, _1));
    m_state_sub =
        m_nh.subscribe<kuam_msgs::Status>("state_machine/status", 10, boost::bind(&Visualizer::StateMachineStatusCallback, this, _1));
    m_global_ego_pos_sub =
        m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&Visualizer::EgoGlobalPosCallback, this, _1));
    m_local_ego_pose_sub = 
        m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&Visualizer::EgoLocalPoseCallback, this, _1));

    // Initialize publisher
    m_setpoints_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("setpoint_markerarray", 10);
    m_ego_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("ego_markerarray", 10);
    m_text_pub = m_p_nh.advertise<jsk_rviz_plugins::OverlayText>("text", 10);
    m_target_height_pub = m_p_nh.advertise<std_msgs::Float32>("/target_height", 10);
    
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
    WINE.r = 255.0/255.0;   WINE.g = 51.0/255.0;    WINE.b = 0.0;
    ORANGE.r = 255.0/255.0; ORANGE.g = 102.0/255.0; ORANGE.b = 0.0;

    //// ego pose
    m_ego_markers.trajectory.ns = "ego/trajectory";
    m_ego_markers.trajectory.id = 0;
    m_ego_markers.trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    m_ego_markers.trajectory.action = visualization_msgs::Marker::ADD;
    m_ego_markers.trajectory.scale.x = 0.05;
    m_ego_markers.trajectory.pose.orientation.w = 1.0;
    m_ego_markers.trajectory.pose.orientation.x = 0.0;
    m_ego_markers.trajectory.pose.orientation.y = 0.0;
    m_ego_markers.trajectory.pose.orientation.z = 0.0;
    m_ego_markers.trajectory.color = CYAN;
    m_ego_markers.trajectory.color.a = 0.4f;
    m_ego_markers.trajectory.lifetime = ros::Duration();
    m_ego_markers.is_trajectory_add = false;

    m_ego_markers.current_point.ns = "ego/current_point";
    m_ego_markers.current_point.id = 0;
    m_ego_markers.current_point.type = visualization_msgs::Marker::MESH_RESOURCE;
    m_ego_markers.current_point.mesh_resource = "package://kuam_state_machine/rviz/meshes/simple_quadcopter/Quadcopter.stl";
    m_ego_markers.current_point.action = visualization_msgs::Marker::ADD;
    m_ego_markers.current_point.scale.x = 0.003;
    m_ego_markers.current_point.scale.y = 0.003;
    m_ego_markers.current_point.scale.z = 0.003;
    m_ego_markers.current_point.color = CYAN;
    m_ego_markers.current_point.color.a = 0.8f;
    m_ego_markers.current_point.lifetime = ros::Duration();
    m_ego_markers.is_current_point_add = false;

    //// text
    m_setpoints_markers.resize((int)SETPOINT::ItemNum);
    m_text_datum.mode = "\n";
    m_text_datum.offb_state = "\n";
    m_text_datum.coverage = "\n";
    m_text_datum.is_marker_detected = "\n";
    m_text_datum.is_vehicle_detected = "\n";
    m_text_datum.landing_state = "\n";
    m_text_datum.ego_global_alt_m = "\n";
    m_text_datum.home_altitude_m = "\n";
    m_text_datum.setpoint_local_h_m = "\n";
    m_text_datum.offset_alt_m = "\n";
}

void Visualizer::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[tf_broadcaster] Home set");
    }
}

void Visualizer::SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr)
{
    kuam_msgs::Setpoint setpoint = *setpoint_ptr; 

    // Set text marker
    m_setpoint_home_altitude_m = setpoint.home_altitude_m;
    m_setpoint_local_height_m = setpoint.local_height_m;
    m_text_datum.home_altitude_m = "Home altitude: " + to_string(m_setpoint_home_altitude_m) + " [m]\n";
    m_text_datum.setpoint_local_h_m = "Local setpoint alt: " + to_string(m_setpoint_local_height_m) + " [m]\n";

    string coverage;
    if (setpoint.is_global) coverage = "GPS coverage";
    else coverage = "Camera coverage";
    m_text_datum.coverage = "Coverage: " + coverage + "\n";
    
    string landing_state = setpoint.landing_state.mode;
    m_text_datum.landing_state = "Landing state: " + landing_state + "\n";

    string is_detected;
    if (setpoint.landing_state.is_marker_detected) is_detected = BoostColor("TRUE", GREEN);
    else is_detected = is_detected = BoostColor("FALSE", RED);
    m_text_datum.is_marker_detected = "Is marker detected: " + is_detected + "\n";

    if (setpoint.landing_state.is_vehicle_detected) is_detected = BoostColor("TRUE", GREEN);
    else is_detected = is_detected = BoostColor("FALSE", RED);
    m_text_datum.is_vehicle_detected = "Is vehicle detected: " + is_detected + "\n";

    float h_m = -setpoint.target_pose.position.z;
    std_msgs::Float32 msg;
    msg.data = h_m;
    m_target_height_pub.publish(msg);

    // Set setpoint marker
    static bool prev_coord = !setpoint.is_global;
    if (setpoint.is_global){
        static geometry_msgs::Point prev_global_point;
        static unsigned int g_cnt = 0;
        bool is_init = false;
        if (prev_coord != setpoint.is_global){
            prev_coord = setpoint.is_global;
            is_init = true;

            //// Setpoints
            visualization_msgs::Marker sp_marker;
            std_msgs::ColorRGBA color = GREEN; color.a = 0.4f;

            sp_marker.header.frame_id = "map";
            sp_marker.ns = "setpoint/global";
            sp_marker.id = g_cnt;
            sp_marker.type = visualization_msgs::Marker::LINE_STRIP;
            sp_marker.action = visualization_msgs::Marker::ADD;
            sp_marker.scale.x = 0.05;
            sp_marker.pose.orientation.w = 1.0;
            sp_marker.pose.orientation.x = 0.0;
            sp_marker.pose.orientation.y = 0.0;
            sp_marker.pose.orientation.z = 0.0;
            sp_marker.color = color;
            sp_marker.lifetime = ros::Duration();

            m_setpoints_markers[(int)SETPOINT::GLOBAL].markers.push_back(sp_marker);

            g_cnt++;
        }

        auto lat = setpoint.geopose.position.latitude;
        auto lon = setpoint.geopose.position.longitude;
        auto alt = setpoint.local_height_m;
        auto p = m_util_visual.ConvertToMapFrame(lat, lon, alt, m_home_position);
        
        float dist = m_util_visual.Distance3D(p.position, prev_global_point);
        if ((dist > 0.02) || is_init){
            m_setpoints_markers[(int)SETPOINT::GLOBAL].markers[g_cnt-1].points.push_back(p.position);
            prev_global_point = p.position;
        }
    }
    else {
        static geometry_msgs::Point prev_local_point;
        static unsigned int r_cnt = 0;
        bool is_init = false;
        if (prev_coord != setpoint.is_global){
            prev_coord = setpoint.is_global;
            is_init = true;
        }

        auto lat = setpoint.geopose.position.latitude;
        auto lon = setpoint.geopose.position.longitude;
        auto alt = setpoint.local_height_m;
        auto p = m_util_visual.ConvertToMapFrame(lat, lon, alt, m_home_position);
        p.orientation = setpoint.geopose.orientation;
        
        float dist = m_util_visual.Distance3D(p.position, prev_local_point);
        if ((dist > 0.02) || is_init){
            auto x = setpoint.vel.linear.x;
            auto y = setpoint.vel.linear.y;
            auto z = setpoint.vel.linear.z;
            auto size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));

            visualization_msgs::Marker sp_marker;
            std_msgs::ColorRGBA color;
            if (setpoint.landing_state.mode == "mode1") color = PURPLE;
            else if (setpoint.landing_state.mode == "mode2") color = WINE;
            color.a = 0.4f;

            sp_marker.header.frame_id = "map";
            sp_marker.header.stamp = ros::Time::now();
            sp_marker.ns = "setpoint/relative";
            sp_marker.id = r_cnt;
            sp_marker.type = visualization_msgs::Marker::ARROW;
            sp_marker.action = visualization_msgs::Marker::ADD;
            sp_marker.scale.x = size;
            sp_marker.scale.y = 0.05;
            sp_marker.scale.z = 0.05;
            sp_marker.pose = p;
            sp_marker.color = color;
            sp_marker.lifetime = ros::Duration();
            m_setpoints_markers[(int)SETPOINT::RELATIVE].markers.push_back(sp_marker);

            prev_local_point = p.position;
            r_cnt++;
        }
    }
    
    visualization_msgs::MarkerArray visualization_markers;
    for (int coord = (int)SETPOINT::GLOBAL; coord < (int)SETPOINT::ItemNum; coord++){
        visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                            m_setpoints_markers[coord].markers.begin(), m_setpoints_markers[coord].markers.end());
    }

    m_setpoints_pub.publish(visualization_markers);
}


void Visualizer::EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr)
{
    static geometry_msgs::Point prev_pos;

    // Set text marker
    auto ego_global_alt_m = ego_ptr->altitude;
    m_text_datum.ego_global_alt_m = "Global altitude: " + to_string(ego_global_alt_m) + " [m]\n";

    float alt_offset_m = ego_global_alt_m - (m_setpoint_local_height_m + m_setpoint_home_altitude_m);
    m_text_datum.offset_alt_m = "Offset altitude: " + to_string(alt_offset_m) + " [m]\n";

    // Set ego markers
    auto lat = ego_ptr->latitude;
    auto lon = ego_ptr->longitude;
    auto alt = m_ego_pose.pose.pose.position.z;
    auto p = m_util_visual.ConvertToMapFrame(lat, lon, alt, m_home_position);

    float dist = m_util_visual.Distance3D(p.position, prev_pos);
    //// trajectory
    if (dist > 0.02){
        m_ego_markers.trajectory.header.frame_id = "map";
        m_ego_markers.trajectory.header.stamp = ros::Time::now();

        m_ego_markers.trajectory.points.push_back(p.position);
        prev_pos = p.position;
    }

    //// current_point
    m_ego_markers.current_point.header.frame_id = "map";
    m_ego_markers.current_point.header.stamp = ros::Time::now();
    m_ego_markers.current_point.pose.position = p.position;
    m_ego_markers.current_point.pose.orientation = m_ego_pose.pose.pose.orientation;

    visualization_msgs::MarkerArray visualization_msg;
    if (m_ego_markers.trajectory.points.size() > 0){
        visualization_msg.markers.push_back(m_ego_markers.trajectory);
    }
    visualization_msg.markers.push_back(m_ego_markers.current_point);

    m_ego_pub.publish(visualization_msg);

    m_text_datum.ego_height_m = to_string(p.position.z) + " [m]\n";
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

    text.text = 
        m_text_datum.mode + m_text_datum.offb_state +
        "\n" +
        m_text_datum.coverage + m_text_datum.is_marker_detected + m_text_datum.is_vehicle_detected + m_text_datum.landing_state +
        "\n" +
        m_text_datum.ego_global_alt_m + m_text_datum.home_altitude_m + m_text_datum.setpoint_local_h_m + m_text_datum.offset_alt_m;
        
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
