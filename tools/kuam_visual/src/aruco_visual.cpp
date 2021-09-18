#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/format.hpp>

#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include "tf2_ros/transform_listener.h" // tf::quaternion
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <kuam_visual/utils.h>
#include <kuam_aruco_tracking/target.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
    // Subscriber
    ros::Subscriber m_aruco_visual_sub;

    // Publisher
    ros::Publisher m_aruco_pub;

    // Param
    string m_data_ns_param;
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

    vector<int> m_marker_ids;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void ArucoVisualCallback(const kuam_msgs::ArucoVisuals::ConstPtr &aruco_msg_ptr);
};


KuamVisualizer::KuamVisualizer() :
    m_data_ns_param("missing"),
    m_small_marker_size_m_param(NAN),
    m_medium_marker_size_m_param(NAN),
    m_big_marker_size_m_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[kuam_visual] Fail GetParam");
    InitROS();

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
    return true;
}

bool KuamVisualizer::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam("/data_ns", m_data_ns_param);
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
    else if (__isnan(m_small_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_small_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_medium_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_medium_marker_size_m_param is NAN"); return false; }
    else if (__isnan(m_big_marker_size_m_param)) { ROS_ERROR_STREAM("[kuam_visual] m_big_marker_size_m_param is NAN"); return false; }

    return true;
}

bool KuamVisualizer::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();

    m_aruco_visual_sub = 
        m_nh.subscribe<kuam_msgs::ArucoVisuals>(m_data_ns_param + "/aruco_tracking/aruco_visuals", 1, boost::bind(&KuamVisualizer::ArucoVisualCallback, this, _1));

    // Initialize publisher
    m_aruco_pub = m_nh.advertise<visualization_msgs::MarkerArray>(nd_name + "/aruco_markerarray", 10);
    
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
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "aruco_visual");
    kuam::KuamVisualizer kuam_visualizer;

    ros::spin();

    return 0;
}