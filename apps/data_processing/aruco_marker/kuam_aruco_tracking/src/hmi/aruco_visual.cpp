#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/format.hpp>

#include <tf/LinearMath/Quaternion.h> // tf::quaternion
#include "tf2_ros/transform_listener.h" // tf::quaternion

#include <kuam_aruco_tracking/utils/util_geometry.h>
#include <kuam_aruco_tracking/target.h>
#include <kuam_msgs/ArucoVisual.h>
#include <kuam_msgs/ArucoVisuals.h>
#include <kuam_msgs/ArucoState.h>
#include <kuam_msgs/ArucoStates.h>
#include <kuam_msgs/FittingPlane.h>
#include <kuam_msgs/FittingPlanes.h>
#include <geometry_msgs/Point.h>
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

class Visualizer
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;
    UtilGeometry m_util_geometry;

public:
    Visualizer();
    ~Visualizer();

private:
    // Subscriber
    ros::Subscriber m_aruco_visual_sub;
    ros::Subscriber m_aruco_states_sub;
    ros::Subscriber m_fitting_planes_sub;

    // Publisher
    ros::Publisher m_aruco_pub;
    ros::Publisher m_landingpoint_pub;
    ros::Publisher m_fitting_plane_pub;

    // Param
    string m_err_param;
    int m_marker_size_type_num_param;

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
    void ArucoStatesCallback(const kuam_msgs::ArucoStates::ConstPtr &aruco_msg_ptr);
    void FittingPlanesCallback(const kuam_msgs::FittingPlanes::ConstPtr &fitting_planes_ptr);
};


Visualizer::Visualizer() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[aruco_visual] Fail GetParam %s", m_err_param.c_str());
    InitROS();
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
    if (!m_p_nh.getParam("marker_size_type_num", m_marker_size_type_num_param))  { m_err_param = "marker_size_type_num"; return false; }

    for (int i = 0; i < m_marker_size_type_num_param; i++){
        string param_name = "marker_ids_type_" + to_string(i);

        vector<int> marker_ids;
        XmlRpc::XmlRpcValue list;
        if (!m_p_nh.getParam(param_name, list))  { m_err_param = param_name; return false; }
        ROS_ASSERT(list.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < list.size(); ++i) {
            ROS_ASSERT(list[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
            int id = static_cast<int>(list[i]);
            m_marker_ids.push_back(id);
        }
    }

    return true;
}

bool Visualizer::InitROS()
{
    // Initialize subscriber
    m_aruco_visual_sub = 
        m_nh.subscribe<kuam_msgs::ArucoVisuals>("aruco_tracking/aruco_visuals", 1, boost::bind(&Visualizer::ArucoVisualCallback, this, _1));
    m_aruco_states_sub = 
        m_nh.subscribe<kuam_msgs::ArucoStates>("aruco_tracking/target_states", 1, boost::bind(&Visualizer::ArucoStatesCallback, this, _1));
    m_fitting_planes_sub = 
        m_nh.subscribe<kuam_msgs::FittingPlanes>("aruco_tracking/fitting_planes", 1, boost::bind(&Visualizer::FittingPlanesCallback, this, _1));

    // Initialize publisher
    m_aruco_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("aruco_markerarray", 10);
    m_landingpoint_pub = m_p_nh.advertise<visualization_msgs::Marker>("landingpoint_marker", 10);
    m_fitting_plane_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("fitting_plane_array", 10);
    
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

void Visualizer::ArucoVisualCallback(const kuam_msgs::ArucoVisuals::ConstPtr &aruco_msg_ptr)
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
                m_targets_markers[ac_visual.id][method].txt.text = "dist: " + m_util_geometry.ToString(m_util_geometry.Distance3D(origin, pos)) + " [m]";
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

void Visualizer::FittingPlanesCallback(const kuam_msgs::FittingPlanes::ConstPtr &fitting_planes_ptr)
{
    visualization_msgs::MarkerArray marker_array;

    for (auto fitting_plane : fitting_planes_ptr->planes){
        if (fitting_plane.is_valid){
            visualization_msgs::Marker plane;
            plane.ns = "fitting_plane/" + to_string(fitting_plane.id);
            plane.header.frame_id = fitting_plane.plane.header.frame_id;
            plane.header.stamp = ros::Time::now();
            plane.type = visualization_msgs::Marker::CUBE;
            plane.action = visualization_msgs::Marker::ADD;
            if (fitting_plane.id == 0){
                plane.scale.x = 2.0;
                plane.scale.y = 2.0;
                plane.scale.z = 0.01;
                plane.color = GREEN;
            }
            else if (fitting_plane.id == 1){
                plane.scale.x = 1.0;
                plane.scale.y = 1.0;
                plane.scale.z = 0.01;
                plane.color = ORANGE;
            }
            else if (fitting_plane.id == 2){
                plane.scale.x = 0.7;
                plane.scale.y = 0.7;
                plane.scale.z = 0.01;
                plane.color = YELLOW;
            }
            plane.color.a = 0.6f;
            plane.pose = fitting_plane.plane.pose;
            plane.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(plane);
        }    
    }
    m_fitting_plane_pub.publish(marker_array);
}

void Visualizer::ArucoStatesCallback(const kuam_msgs::ArucoStates::ConstPtr &aruco_msg_ptr)
{
    if (aruco_msg_ptr->is_detected){
        visualization_msgs::Marker landing_point;
        landing_point.header.frame_id = aruco_msg_ptr->header.frame_id;
        landing_point.header.stamp = ros::Time::now();
        landing_point.ns = "setpoint/landing_point";
        landing_point.id = 0;
        landing_point.type = visualization_msgs::Marker::SPHERE;
        landing_point.action = visualization_msgs::Marker::ADD;
        landing_point.scale.x = 0.15;
        landing_point.scale.y = 0.15;
        landing_point.scale.z = 0.15;
        landing_point.pose = aruco_msg_ptr->target_pose;
        landing_point.color = ORANGE;
        landing_point.color.a = 1.0f;
        landing_point.lifetime = ros::Duration(0.1);

        m_landingpoint_pub.publish(landing_point);
    }
}
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "aruco_visual");
    kuam::Visualizer kuam_visualizer;

    ros::spin();

    return 0;
}