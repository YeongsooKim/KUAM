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
#include <std_msgs/Float32.h>
#include <kuam_visual/utils.h>
#include <kuam_aruco_tracking/target.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>
#include <kuam_msgs/Waypoints.h>
#include <kuam_msgs/Setpoint.h>
#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/TransReq.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPose.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/HomePosition.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <sensor_msgs/BatteryState.h>

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
    string cur_mode;
    string cur_state;
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

class KuamVisualizer
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    Utils m_utils;
    TextDatum m_text_datum;

public:
    KuamVisualizer();
    ~KuamVisualizer();

private:
    // Subscriber
    ros::Subscriber m_aruco_visual_sub;
    ros::Subscriber m_aruco_state_sub;
    ros::Subscriber m_home_position_sub;
    ros::Subscriber m_global_waypoints_sub;
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_local_ego_pose_sub;
    ros::Subscriber m_global_ego_pos_sub;
    ros::Subscriber m_state_sub;
    ros::Subscriber m_tasklist_sub;
    ros::Subscriber m_battery_sub;

    // Publisher
    ros::Publisher m_aruco_pub;
    ros::Publisher m_global_path_pub;
    ros::Publisher m_setpoints_pub;
    ros::Publisher m_ego_pub;
    ros::Publisher m_text_pub;
    ros::Publisher m_gt_marker_pub;
    ros::Publisher m_gt_pos_pub;
    ros::Publisher m_target_height_pub;

    // Timer
    ros::Timer m_text_pub_cb_timer;
    ros::Timer m_gt_pub_cb_timer;

    // Param
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

    // Flag
    bool m_is_home_set;
    bool m_is_init_global_path;

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
    
    void ArucoVisualCallback(const kuam_msgs::ArucoVisuals::ConstPtr &aruco_msg_ptr);
    void ArucoStateCallback(const kuam_msgs::ArucoStates::ConstPtr &aruco_msg_ptr);
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);
    void WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr);
    void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr);
    inline void EgoLocalPoseCallback(const nav_msgs::Odometry::ConstPtr &ego_ptr) { m_ego_pose = *ego_ptr; }
    void EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr);
    void StateCallback(const kuam_msgs::TransReq::ConstPtr &state_ptr);
    void TaskListCallback(const kuam_msgs::TaskList::ConstPtr &ego_ptr);
    void BatteryCallback(const sensor_msgs::BatteryState::ConstPtr &battery_ptr);
    void TextPubCallback(const ros::TimerEvent& event);
    void GTPubCallback(const ros::TimerEvent& event);
};


KuamVisualizer::KuamVisualizer() :
    m_data_ns_param("missing"),
    m_maneuver_ns_param("missing"),
    m_standby_dist_th_m_param(NAN),
    m_landing_standby_alt_m_param(NAN),
    m_small_marker_size_m_param(NAN),
    m_medium_marker_size_m_param(NAN),
    m_big_marker_size_m_param(NAN),
    m_tfListener(m_tfBuffer)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[kuam_visual] Fail GetParam");
    InitROS();

    m_setpoints.resize((int)SETPOINT::ItemNum);

    for (auto id : m_big_marker_ids_param) m_marker_ids.push_back(id);
    for (auto id : m_medium_marker_ids_param) m_marker_ids.push_back(id);
    for (auto id : m_small_marker_ids_param) m_marker_ids.push_back(id);
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

    m_nh.getParam("/data_ns", m_data_ns_param);
    m_nh.getParam("/maneuver_ns", m_maneuver_ns_param);
    m_nh.getParam(m_maneuver_ns_param + "/state_machine/standby_dist_th_m", m_standby_dist_th_m_param);
    m_nh.getParam(m_maneuver_ns_param + "/state_machine/landing_standby_alt_m", m_landing_standby_alt_m_param);
    m_nh.getParam(m_data_ns_param + "/aruco_tracking/small_marker_size_m", m_small_marker_size_m_param);
    m_nh.getParam(m_data_ns_param + "/aruco_tracking/medium_marker_size_m", m_medium_marker_size_m_param);
    m_nh.getParam(m_data_ns_param + "/aruco_tracking/big_marker_size_m", m_big_marker_size_m_param);
    XmlRpc::XmlRpcValue list;
    m_nh.getParam(m_data_ns_param + "/aruco_tracking/big_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_big_marker_ids_param.push_back(id);
    }
    m_nh.getParam(m_data_ns_param + "/aruco_tracking/medium_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_medium_marker_ids_param.push_back(id);
    }

    m_nh.getParam(m_data_ns_param + "/aruco_tracking/small_marker_ids", list);
    ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < list.size(); ++i) {
        ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
        int id = static_cast<int>(list[i]);
        m_small_marker_ids_param.push_back(id);
    }

    if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("[kuam_visual] m_data_ns_param is missing"); return false; }
    else if (m_maneuver_ns_param == "missing") { ROS_ERROR_STREAM("[kuam_visual] m_maneuver_ns_param is missing"); return false; }
    else if (__isnan(m_standby_dist_th_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_standby_dist_th_m_param is NAN"); return false; }
    else if (__isnan(m_landing_standby_alt_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_landing_standby_alt_m_param is NAN"); return false; }
    else if (__isnan(m_small_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_small_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_medium_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_medium_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_big_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_big_marker_size_m_param is NAN"); return false; }

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
        m_nh.subscribe<kuam_msgs::ArucoVisuals>(m_data_ns_param + "/aruco_tracking/aruco_visuals", 1, boost::bind(&KuamVisualizer::ArucoVisualCallback, this, _1));
    m_aruco_state_sub = 
        m_nh.subscribe<kuam_msgs::ArucoStates>(m_data_ns_param + "/aruco_tracking/target_states", 10, boost::bind(&KuamVisualizer::ArucoStateCallback, this, _1));        
    m_global_waypoints_sub = 
        m_nh.subscribe<kuam_msgs::Waypoints>(m_data_ns_param + "/csv_parser/waypoints", 10, boost::bind(&KuamVisualizer::WaypointsCallback, this, _1));
    m_setpoint_sub = 
        m_nh.subscribe<kuam_msgs::Setpoint>(m_maneuver_ns_param + "/state_machine/setpoint", 10, boost::bind(&KuamVisualizer::SetpointCallback, this, _1));
    m_local_ego_pose_sub = 
        m_nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 10, boost::bind(&KuamVisualizer::EgoLocalPoseCallback, this, _1));
    m_global_ego_pos_sub =
        m_nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 10, boost::bind(&KuamVisualizer::EgoGlobalPosCallback, this, _1));
    m_state_sub =
        m_nh.subscribe<kuam_msgs::TransReq>(m_maneuver_ns_param + "/state_machine/trans_request", 10, boost::bind(&KuamVisualizer::StateCallback, this, _1));
    m_tasklist_sub = 
        m_nh.subscribe<kuam_msgs::TaskList>(m_maneuver_ns_param + "/mission_manager/tasklist", 10, boost::bind(&KuamVisualizer::TaskListCallback, this, _1));
    m_battery_sub = 
        m_nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery", 10, boost::bind(&KuamVisualizer::BatteryCallback, this, _1));

    // Initialize publisher
    m_aruco_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/aruco_markerarray", 10);
    m_global_path_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/global_path_markerarray", 10);
    m_setpoints_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/setpoint_markerarray", 10);
    m_ego_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/ego_markerarray", 10);
    m_text_pub = m_nh.advertise<jsk_rviz_plugins::OverlayText>(nd_name + "/text", 10);
    m_gt_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/gt_markerarray", 10);
    m_gt_pos_pub = m_nh.advertise<geometry_msgs::PoseArray>(nd_name + "/gt_pos", 10);
    m_target_height_pub = m_nh.advertise<std_msgs::Float32>(nd_name + "/target_height", 10);
    
    // Initialize timer
    float freq = 5.0;
    m_text_pub_cb_timer = m_nh.createTimer(ros::Duration(1.0/freq), &KuamVisualizer::TextPubCallback, this);
    m_gt_pub_cb_timer = m_nh.createTimer(ros::Duration(1.0/freq), &KuamVisualizer::GTPubCallback, this);

    return true;
}

bool KuamVisualizer::InitMarkers()
{
    RED.r = 1.0f;       RED.g = 0.0f;       RED.b = 0.0f;
    GREEN.r = 0.0f;     GREEN.g = 1.0f;     GREEN.b = 0.0f;
    BLUE.r = 0.0f;      BLUE.g = 0.0f;      BLUE.b = 1.0f;
    YELLOW.r = 1.0f;    YELLOW.g = 1.0f;    YELLOW.b = 0.0f;
    WHITE.r = 1.0f;     WHITE.g = 1.0f;     WHITE.b = 1.0f;
    PURPLE.r = 1.0f;    PURPLE.g = 0.0f;    PURPLE.b = 1.0f;
    CYAN.r = 0.0f;      CYAN.g = 1.0f;      CYAN.b = 1.0f;
    ORANGE.r = 255.0/255.0; ORANGE.g = 102.0/255.0; ORANGE.b = 0.0;

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
    m_global_markers.markers.push_back(path);

    //// setpoint
    m_setpoints_markers.resize((int)SETPOINT::ItemNum);

    //// target aruco marker
    for (auto id : m_marker_ids){
        vector<MetaMarkers> target_markers;
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            MetaMarkers target_marker;
            // Without filter. GREEN
            target_marker.trajectory.ns = "target" + to_string(id) + "/trajectory";
            target_marker.trajectory.type = visualization_msgs::Marker::LINE_STRIP;
            target_marker.trajectory.action = visualization_msgs::Marker::ADD;
            target_marker.trajectory.scale.x = 0.05;
            target_marker.trajectory.pose.orientation.w = 1.0;
            target_marker.trajectory.pose.orientation.x = 0.0;
            target_marker.trajectory.pose.orientation.y = 0.0;
            target_marker.trajectory.pose.orientation.z = 0.0;
            target_marker.trajectory.lifetime = ros::Duration(0.1);
            target_marker.is_trajectory_add = false;

            target_marker.current_point.ns = "target" + to_string(id) + "/current_point";
            target_marker.current_point.type = visualization_msgs::Marker::SPHERE;
            target_marker.current_point.action = visualization_msgs::Marker::ADD;
            target_marker.current_point.scale.x = 0.08;
            target_marker.current_point.scale.y = 0.08;
            target_marker.current_point.scale.z = 0.08;
            target_marker.current_point.pose.orientation.w = 1.0;
            target_marker.current_point.pose.orientation.x = 0.0;
            target_marker.current_point.pose.orientation.y = 0.0;
            target_marker.current_point.pose.orientation.z = 0.0;
            target_marker.current_point.lifetime = ros::Duration(0.1);
            target_marker.is_current_point_add = false;
            
            target_marker.txt.ns = "target" + to_string(id) + "/txt";
            target_marker.txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            target_marker.txt.action = visualization_msgs::Marker::ADD;
            target_marker.txt.scale.z = 0.07;
            target_marker.txt.pose.orientation.w = 1.0;
            target_marker.txt.pose.orientation.x = 0.0;
            target_marker.txt.pose.orientation.y = 0.0;
            target_marker.txt.pose.orientation.z = 0.0;
            target_marker.txt.lifetime = ros::Duration(0.1);
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
            target_markers.push_back(target_marker);
        }

        m_targets_markers.insert(pair< int, aruco >(id, target_markers));
    }

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
    m_ego_markers.current_point.mesh_resource = "package://kuam_visual/meshes/simple_quadcopter/Quadcopter.stl";
    m_ego_markers.current_point.action = visualization_msgs::Marker::ADD;
    m_ego_markers.current_point.scale.x = 0.003;
    m_ego_markers.current_point.scale.y = 0.003;
    m_ego_markers.current_point.scale.z = 0.003;
    m_ego_markers.current_point.color = CYAN;
    m_ego_markers.current_point.color.a = 0.8f;
    m_ego_markers.current_point.lifetime = ros::Duration();
    m_ego_markers.is_current_point_add = false;
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

void KuamVisualizer::ArucoVisualCallback(const kuam_msgs::ArucoVisuals::ConstPtr &aruco_msg_ptr)
{
    geometry_msgs::Point pos;
    geometry_msgs::Point origin;
    for (auto ac_visual : aruco_msg_ptr->aruco_visuals){
        if (!ac_visual.is_detected){
            continue;
        }
        
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            if (ac_visual.is_compare_mode || (method == ac_visual.esti_method)){
                // current_point
                pos = ac_visual.poses[method].position;

                m_targets_markers[ac_visual.id][method].current_point.header.frame_id = ac_visual.header.frame_id;
                m_targets_markers[ac_visual.id][method].current_point.header.stamp = ros::Time(0);

                m_targets_markers[ac_visual.id][method].current_point.pose.position = pos;
                m_targets_markers[ac_visual.id][method].is_current_point_add = true;

                // txt
                m_targets_markers[ac_visual.id][method].txt.header.frame_id = ac_visual.header.frame_id;;
                m_targets_markers[ac_visual.id][method].txt.header.stamp = ros::Time(0);

                m_targets_markers[ac_visual.id][method].txt.pose.position = pos;
                m_targets_markers[ac_visual.id][method].txt.pose.position.z += 0.2;
                m_targets_markers[ac_visual.id][method].txt.text = "dist: " + m_utils.ToString(m_utils.Distance3D(origin, pos)) + " [m]";
                m_targets_markers[ac_visual.id][method].is_txt_add = true;
            }
        }
    }

    for (auto ac_visual : aruco_msg_ptr->aruco_visuals){
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            m_targets_markers[ac_visual.id][method].self.markers.clear();
            if (m_targets_markers[ac_visual.id][method].is_current_point_add) m_targets_markers[ac_visual.id][method].self.markers.push_back(m_targets_markers[ac_visual.id][method].current_point);
            if (m_targets_markers[ac_visual.id][method].is_txt_add) m_targets_markers[ac_visual.id][method].self.markers.push_back(m_targets_markers[ac_visual.id][method].txt);
            m_targets_markers[ac_visual.id][method].is_current_point_add = m_targets_markers[ac_visual.id][method].is_txt_add = false;
        }
    }
    
    visualization_msgs::MarkerArray visualization_markers;
    for (auto ac_visual : aruco_msg_ptr->aruco_visuals){
        for (int method = (int)EstimatingMethod::WOF; method < (int)EstimatingMethod::ItemNum; method++){
            visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                                m_targets_markers[ac_visual.id][method].self.markers.begin(), m_targets_markers[ac_visual.id][method].self.markers.end());
        }
    }

    m_aruco_pub.publish(visualization_markers);
}

void KuamVisualizer::ArucoStateCallback(const kuam_msgs::ArucoStates::ConstPtr &aruco_msg_ptr)
{
    vector<float> heights;
    for (auto ac_state : aruco_msg_ptr->aruco_states){
        if (ac_state.is_detected){
            geometry_msgs::TransformStamped transformStamped;
            geometry_msgs::Pose transformed_pose;
            try{
                transformStamped = 
                        m_tfBuffer.lookupTransform("base_link", ac_state.header.frame_id, ros::Time(0));
                        tf2::doTransform(ac_state.pose, transformed_pose, transformStamped);
                }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
            }
            transformed_pose;
            if (m_utils.IsValid(transformed_pose)){
                heights.push_back(abs(transformed_pose.position.z));
            }
        }
    }

    if (heights.empty()){
        return;
    }

    float h_sum_m = 0.0;
    for (auto h : heights){
        h_sum_m += h;
    }
    float h_m = h_sum_m/heights.size();
    std_msgs::Float32 msg;
    msg.data = h_m;
    m_target_height_pub.publish(msg);
}

void KuamVisualizer::WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr)
{
    // if (!m_is_init_global_path){
    //     m_is_init_global_path = true;

    //     // Get pose
    //     vector<geometry_msgs::Point> points;
    //     for (auto wp : wps_ptr->waypoints){
    //         auto lat = wp.geopose.position.latitude;
    //         auto lon = wp.geopose.position.longitude;
    //         auto alt = wp.geopose.position.altitude;
    //         auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
    //         points.push_back(p);
    //     }

    //     m_global_markers.markers[0].header.stamp = ros::Time::now();
    //     m_global_markers.markers[0].points = points;

    //     // Allocate orientation
    //     vector<geometry_msgs::Pose> poses;
    //     for (int i = 0; i < points.size(); i++){
    //         geometry_msgs::Pose p;
    //         if (points.size() != 1){
    //             if (i + 1 < points.size()){
    //                 auto orientation = m_utils.GetOrientation(points[i], points[i+1]);
    //                 p.orientation = orientation;
    //                 p.position = points[i];
    //             }
    //             else {
    //                 p.orientation = poses[i-1].orientation;
    //                 p.position = poses[i-1].position;
    //             }
    //             poses.push_back(p);
    //         }
    //         else{
    //             p.orientation.x = 0.0;
    //             p.orientation.y = 0.0;
    //             p.orientation.z = 0.0;
    //             p.orientation.w = 1.0;
    //             p.position = points[i];
    //             poses.push_back(p);
    //         }
    //     }

    //     for (int i = 0; i < poses.size(); i++){

    //         visualization_msgs::Marker yaw;
    //         yaw.header.frame_id = "map";
    //         yaw.header.stamp = ros::Time::now();
    //         yaw.ns = "global/yaw";
    //         yaw.id = i;
    //         yaw.type = visualization_msgs::Marker::ARROW;
    //         yaw.action = visualization_msgs::Marker::ADD;
    //         yaw.scale.x = 0.3;
    //         yaw.scale.y = 0.1;
    //         yaw.scale.z = 0.1;
    //         yaw.pose = poses[i];
    //         yaw.color = RED;
    //         yaw.color.a = 0.4f;
    //         yaw.lifetime = ros::Duration();
    //         m_global_markers.markers.push_back(yaw);
    //     }

    //     int ld_index = 0;
    //     bool has_landingpoint = false;
    //     for (int i = 0; i < wps_ptr->waypoints.size(); i++){
    //         if (wps_ptr->waypoints[i].mission == "landing"){
    //             ld_index = i;
    //             has_landingpoint = true;
    //         }
    //     }

    //     if (has_landingpoint){
    //         auto geopose = wps_ptr->waypoints[ld_index].geopose;
    //         auto lat = geopose.position.latitude;
    //         auto lon = geopose.position.longitude;
    //         auto alt = m_landing_standby_alt_m_param;
    //         auto landing_pos = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);

    //         visualization_msgs::Marker landing_point;
    //         landing_point.header.frame_id = "map";
    //         landing_point.header.stamp = ros::Time::now();
    //         landing_point.ns = "global/landing_point";
    //         landing_point.id = 0;
    //         landing_point.type = visualization_msgs::Marker::SPHERE;
    //         landing_point.action = visualization_msgs::Marker::ADD;
    //         landing_point.scale.x = m_standby_dist_th_m_param;
    //         landing_point.scale.y = m_standby_dist_th_m_param;
    //         landing_point.scale.z = m_standby_dist_th_m_param;
    //         landing_point.pose.position = landing_pos;
    //         landing_point.pose.orientation.x = 0.0;
    //         landing_point.pose.orientation.y = 0.0;
    //         landing_point.pose.orientation.z = 0.0;
    //         landing_point.pose.orientation.w = 1.0;
    //         landing_point.color = GREEN;
    //         landing_point.color.a = 0.3f;
    //         landing_point.lifetime = ros::Duration();
    //         m_global_markers.markers.push_back(landing_point);

    //         //// ground truth
    //         visualization_msgs::Marker gt_aruco;
    //         gt_aruco.header.frame_id = "map";
    //         gt_aruco.type = visualization_msgs::Marker::CUBE;
    //         gt_aruco.action = visualization_msgs::Marker::ADD;
    //         gt_aruco.pose.orientation.w = 1.0;
    //         gt_aruco.pose.orientation.x = 0.0;
    //         gt_aruco.pose.orientation.y = 0.0;
    //         gt_aruco.pose.orientation.z = 0.0;
    //         gt_aruco.lifetime = ros::Duration();

    //         auto small_gt_pos = landing_pos;
    //         small_gt_pos.x += 0.30;
    //         small_gt_pos.y -= 0.30;
    //         small_gt_pos.z = 0.0; // ground
    //         gt_aruco.ns = "gt/small";
    //         gt_aruco.id = 0;
    //         gt_aruco.scale.x = m_medium_marker_size_m_param;
    //         gt_aruco.scale.y = m_medium_marker_size_m_param;
    //         gt_aruco.scale.z = 0.01;
    //         gt_aruco.pose.position = small_gt_pos;
    //         gt_aruco.color = GREEN;
    //         gt_aruco.color.a = 1.0f;
    //         m_gt_markers.markers.push_back(gt_aruco);

    //         auto big_gt_pos = landing_pos;
    //         // big_gt_pos.x -= m_big_marker_size_m_param/2.0;
    //         // big_gt_pos.y -= (m_big_marker_size_m_param + m_medium_marker_size_m_param)/2.0;
    //         big_gt_pos.z = 0.0;
    //         gt_aruco.ns = "gt/big";
    //         gt_aruco.id = 1;
    //         gt_aruco.scale.x = m_big_marker_size_m_param;
    //         gt_aruco.scale.y = m_big_marker_size_m_param;
    //         gt_aruco.scale.z = 0.01;
    //         gt_aruco.pose.position = big_gt_pos;
    //         gt_aruco.color = YELLOW;
    //         gt_aruco.color.a = 1.0f;
    //         m_gt_markers.markers.push_back(gt_aruco);
    //     }
    // }
    // else {
    //     m_global_path_pub.publish(m_global_markers);
    // }
}

void KuamVisualizer::SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr)
{
    kuam_msgs::Setpoint setpoint = *setpoint_ptr; 

    // Set text marker
    m_text_datum.home_altitude_m = to_string(setpoint.home_altitude_m);
    m_text_datum.setpoint_local_h_m = to_string(setpoint.local_height_m);

    if (setpoint.is_global){
        m_text_datum.coverage = "GPS coverage\n";
    }
    else{
        m_text_datum.coverage = "Camera coverage\n";
    }

    if (m_text_datum.cur_state == "LANDING\n"){
        if (!setpoint.landing_state.is_pass_landing_standby){
            m_text_datum.landing_state = "Standby\n";
        }
        else {
            if (!setpoint.landing_state.is_land){
                m_text_datum.landing_state = "KUAM ctrl\n";
            }
            else {
                m_text_datum.landing_state = "FC ctrl\n";
            }
        }
    }
    else {
        m_text_datum.landing_state = "Not in landing\n";
    }

    string is_detected;
    if (setpoint.landing_state.is_detected){
        is_detected = "TRUE\n";
        m_text_datum.is_detected = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % is_detected % 0.0 % 255.0 % 0.0 % 1.0).str();
    }
    else{
        is_detected = "FALSE\n";
        m_text_datum.is_detected = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % is_detected % 255.0 % 0.0 % 0.0 % 1.0).str();
    }
    string is_pass_landing_standby;
    if (setpoint.landing_state.is_pass_landing_standby){
        is_pass_landing_standby = "TRUE\n";
        m_text_datum.is_pass_landing_standby = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % is_pass_landing_standby % 0.0 % 255.0 % 0.0 % 1.0).str();
    }
    else{
        is_pass_landing_standby = "FALSE\n";
        m_text_datum.is_pass_landing_standby = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % is_pass_landing_standby % 255.0 % 0.0 % 0.0 % 1.0).str();
    }


    // Set setpoint marker
    static bool prev_coord = !setpoint.is_global;
    if (setpoint.is_global){
        static geometry_msgs::Point prev_global_point;
        static unsigned int g_cnt = 0;
        bool is_init = false;
        if (prev_coord != setpoint.is_global){
            prev_coord = setpoint.is_global;
            is_init = true;
            g_cnt++;

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
        }

        auto geopos = setpoint.geopose.position;
        auto lat = geopos.latitude;
        auto lon = geopos.longitude;
        auto alt = setpoint.local_height_m;
        auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
        
        float dist = m_utils.Distance3D(p, prev_global_point);
        if ((dist > 0.02) || is_init){
            m_setpoints_markers[(int)SETPOINT::GLOBAL].markers[g_cnt-1].points.push_back(p);
            prev_global_point = p;
        }
    }
    else {
        static geometry_msgs::Point prev_pos;
        static unsigned int r_cnt = 0;
        bool is_init = false;
        if (prev_coord != setpoint.is_global){
            prev_coord = setpoint.is_global;
            is_init = true;
        }

        float dist = m_utils.Distance3D(m_ego_markers.current_point.pose.position, prev_pos);
        if ((dist > 0.02) || is_init){
            auto x = setpoint.vel.linear.x;
            auto y = setpoint.vel.linear.y;
            auto z = setpoint.vel.linear.z;
            auto size = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));

            visualization_msgs::Marker sp_marker;
            std_msgs::ColorRGBA color = PURPLE; color.a = 0.4f;
            sp_marker.header.frame_id = "map";
            sp_marker.header.stamp = ros::Time::now();
            sp_marker.ns = "setpoint/relative";
            sp_marker.id = r_cnt;
            sp_marker.type = visualization_msgs::Marker::ARROW;
            sp_marker.action = visualization_msgs::Marker::ADD;
            sp_marker.scale.x = size;
            sp_marker.scale.y = 0.05;
            sp_marker.scale.z = 0.05;
            sp_marker.pose.position = m_ego_markers.current_point.pose.position;
            sp_marker.pose.orientation = setpoint.pose.orientation;
            sp_marker.color = color;
            sp_marker.lifetime = ros::Duration();
            m_setpoints_markers[(int)SETPOINT::RELATIVE].markers.push_back(sp_marker);

            prev_pos = m_ego_markers.current_point.pose.position;
            r_cnt++;
        }
    }
    
    visualization_msgs::MarkerArray visualization_markers;
    for (int coord = (int)SETPOINT::GLOBAL; coord < (int)SETPOINT::ItemNum; coord++){
        visualization_markers.markers.insert(visualization_markers.markers.end(), 
                                            m_setpoints_markers[coord].markers.begin(), m_setpoints_markers[coord].markers.end());
    }

    if (!setpoint.is_global){
        geometry_msgs::Pose p;
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::Pose transformed_pose;
        try{
            transformStamped = 
                m_tfBuffer.lookupTransform("map", "landing_point", ros::Time(0));
            tf2::doTransform(p, transformed_pose, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }

        visualization_msgs::Marker landing_point;
        landing_point.header.frame_id = "map";
        landing_point.header.stamp = ros::Time::now();
        landing_point.ns = "setpoint/landing_point";
        landing_point.id = 0;
        landing_point.type = visualization_msgs::Marker::SPHERE;
        landing_point.action = visualization_msgs::Marker::ADD;
        landing_point.scale.x = 0.15;
        landing_point.scale.y = 0.15;
        landing_point.scale.z = 0.15;
        landing_point.pose.position = transformed_pose.position;
        landing_point.pose.orientation.x = 0.0;
        landing_point.pose.orientation.y = 0.0;
        landing_point.pose.orientation.z = 0.0;
        landing_point.pose.orientation.w = 1.0;
        landing_point.color = ORANGE;
        landing_point.color.a = 1.0f;
        landing_point.lifetime = ros::Duration(0.1);
        visualization_markers.markers.push_back(landing_point);
    }
    m_setpoints_pub.publish(visualization_markers);
}

void KuamVisualizer::EgoGlobalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &ego_ptr)
{
    static geometry_msgs::Point prev_pos;

    // Set text marker
    auto ego_global_alt_m = ego_ptr->altitude;

    float setpoint_local_alt_m;
    if (m_text_datum.setpoint_local_h_m == "") setpoint_local_alt_m = 0;
    else setpoint_local_alt_m = stof(m_text_datum.setpoint_local_h_m);

    float home_altitude_m;
    if (m_text_datum.home_altitude_m == "") home_altitude_m = 0;
    else home_altitude_m = stof(m_text_datum.home_altitude_m);
    
    m_text_datum.ego_global_alt_m = to_string(ego_global_alt_m);

    float alt_offset_m = ego_global_alt_m - (setpoint_local_alt_m + home_altitude_m);
    m_text_datum.offset_alt_m = to_string(alt_offset_m);

    // Set ego markers
    auto lat = ego_ptr->latitude;
    auto lon = ego_ptr->longitude;
    auto alt = m_ego_pose.pose.pose.position.z;
    auto p = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);

    float dist = m_utils.Distance3D(p, prev_pos);
    //// trajectory
    if (dist > 0.02){
        m_ego_markers.trajectory.header.frame_id = "map";
        m_ego_markers.trajectory.header.stamp = ros::Time::now();

        m_ego_markers.trajectory.points.push_back(p);
        prev_pos = p;
    }

    //// current_point
    m_ego_markers.current_point.header.frame_id = "map";
    m_ego_markers.current_point.header.stamp = ros::Time::now();
    m_ego_markers.current_point.pose.position = p;
    m_ego_markers.current_point.pose.orientation = m_ego_pose.pose.pose.orientation;

    visualization_msgs::MarkerArray visualization_msg;
    if (m_ego_markers.trajectory.points.size() > 0){
        visualization_msg.markers.push_back(m_ego_markers.trajectory);
    }
    visualization_msg.markers.push_back(m_ego_markers.current_point);

    m_ego_pub.publish(visualization_msg);

    m_text_datum.ego_height_m = to_string(p.z) + "[m]\n";
}

void KuamVisualizer::StateCallback(const kuam_msgs::TransReq::ConstPtr &state_ptr)
{
    string state = state_ptr->state + "\n";
    string mode = state_ptr->mode.kuam + "\n";
    if (mode == "EMERGY\n"){
        m_text_datum.cur_mode = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % mode % 255.0 % 0.0 % 0.0 % 1.0).str();
    }
    else{
        m_text_datum.cur_mode = mode;
    }

    m_text_datum.cur_state = state;    
}

void KuamVisualizer::TaskListCallback(const kuam_msgs::TaskList::ConstPtr &tasklist_ptr)
{
    string lb = "\n";
    string tab = "\t";
    stringstream ss;

    auto task_list = tasklist_ptr->task;
    auto status_list = tasklist_ptr->status;

    for (int i = 0; i < task_list.size(); i++){
        ss << task_list[i] << ":\t" << status_list[i] << lb;
    }
    m_text_datum.tasklist = ss.str();
}

void KuamVisualizer::BatteryCallback(const sensor_msgs::BatteryState::ConstPtr &battery_ptr)
{
    string battery_per = to_string(battery_ptr->percentage);
    if (battery_ptr->percentage == 0.0){
        m_text_datum.battery_per = "0.0\n";
    }
    else if (battery_ptr->percentage < 0.10){
        m_text_datum.battery_per = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % battery_per % 255.0 % 0.0 % 0.0 % 1.0).str();
    }
    else if (battery_ptr->percentage < 0.30){
        m_text_datum.battery_per = (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>")
             % battery_per % 255.0 % 204.0 % 0.0 % 1.0).str();
    }
    else{
        m_text_datum.battery_per = battery_per;
    }
}

void KuamVisualizer::TextPubCallback(const ros::TimerEvent& event)
{
    jsk_rviz_plugins::OverlayText text;
    std_msgs::ColorRGBA fg_color;   fg_color.r = 25.0/255.0;    fg_color.g = 1.0;           fg_color.b = 240.0/255.0;   fg_color.a = 1.0;
    std_msgs::ColorRGBA bg_color;   bg_color.r = 136.0/255.0;   bg_color.g = 138.0/255.0;   bg_color.b = 133.0/255.0;   bg_color.a = 0.35;

    text.width = 350;
    text.height = 800;
    text.left = 10;
    text.top = 10;
    text.text_size = 12;
    text.line_width = 2;
    text.font = "DejaVu Sans Mono";
    auto coverage = m_text_datum.coverage;
    auto is_detected = m_text_datum.is_detected;
    auto is_pass_landing_standby = m_text_datum.is_pass_landing_standby;
    auto ego_local_height_m = m_text_datum.ego_height_m;
    auto cur_mode = m_text_datum.cur_mode;
    auto cur_state = m_text_datum.cur_state;
    auto landing_state = m_text_datum.landing_state;
    auto tasklist = m_text_datum.tasklist;
    auto home_altitude_m =  m_text_datum.home_altitude_m;
    auto setpoint_local_h_m =  m_text_datum.setpoint_local_h_m;
    auto offset_alt_m =  m_text_datum.offset_alt_m;
    auto ego_global_alt_m = m_text_datum.ego_global_alt_m;
    auto battery_per = m_text_datum.battery_per;
    
    text.text = 
        "Coverage: " + coverage + "Is detected: " + is_detected + "Is pass landing standby: " + is_pass_landing_standby +
        "\nLocal altitude: " + ego_local_height_m + 
        "\nCurrent mode: " + cur_mode + "Current state: " + 
        cur_state + "\nLanding state: " + 
        landing_state + "\nTask list: \n" + tasklist +
        "\n---\n\nGlobal altitude: " + ego_global_alt_m + "[m]\nHome altitude: " + home_altitude_m + "[m]\nLocal setpoint alt: " + setpoint_local_h_m + "[m]\nOffset altitude: " + offset_alt_m + "[m]\n" +
        "\nBattery percentage: " + battery_per;
    text.fg_color = fg_color;
    text.bg_color = bg_color;

    m_text_pub.publish(text);
}

void KuamVisualizer::GTPubCallback(const ros::TimerEvent& event)
{
    if (!m_gt_markers.markers.empty()){
        m_gt_marker_pub.publish(m_gt_markers);

        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        pose_array.header.stamp = ros::Time::now();

        for (auto marker : m_gt_markers.markers){
            geometry_msgs::Pose p;
            p = marker.pose;

            pose_array.poses.push_back(p);
        }
        reverse(pose_array.poses.begin(), pose_array.poses.end());

        m_gt_pos_pub.publish(pose_array);
    }
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
