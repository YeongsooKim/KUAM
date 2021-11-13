#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>

#include <tf/LinearMath/Quaternion.h> // tf::quaternion

#include <kuam_msgs/Waypoint.h>
#include <kuam_msgs/Waypoints.h>

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

class Visualizer
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;

public:
    Visualizer();
    ~Visualizer();

private:
    // Subscriber
    ros::Subscriber m_global_path_sub;

    // Publisher
    ros::Publisher m_global_marker_pub;

    // Param
    string m_err_param;

    // Flag
    
    // Markers
    visualization_msgs::MarkerArray m_global_markers;

    vector<geometry_msgs::Pose> m_global_path;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();
    
    void GlobalPathCallback(const kuam_msgs::Waypoints::ConstPtr &wp_msg_ptr);
};


Visualizer::Visualizer() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[global_path_visual] Fail GetParam %s", m_err_param);
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
    return true;
}

bool Visualizer::InitROS()
{
    // Initialize subscriber
    m_global_path_sub = 
        m_nh.subscribe<kuam_msgs::Waypoints>("global_path/waypoints", 1, boost::bind(&Visualizer::GlobalPathCallback, this, _1));

    // Initialize publisher
    m_global_marker_pub = m_p_nh.advertise<visualization_msgs::MarkerArray>("global_path_markerarray", 10);
    
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

}

void Visualizer::GlobalPathCallback(const kuam_msgs::Waypoints::ConstPtr &wp_msg_ptr)
{
    if (wp_msg_ptr->waypoints.empty()) return;

    static string mission = "none";
    static string prev_mission = "none";
    kuam_msgs::Waypoint waypoint;
    for (auto wp : wp_msg_ptr->waypoints){
        if (wp.poses.empty()) {
            m_global_marker_pub.publish(m_global_markers);

            return;
        }

        if (mission != wp.mission){
            mission = wp.mission;
            waypoint = wp;
            break;
        }
    }

    vector<geometry_msgs::Pose> poses;
    if (mission != prev_mission){
        m_global_path.insert(m_global_path.end(), waypoint.poses.begin(), waypoint.poses.end());
        // global path strip
        vector<geometry_msgs::Point> points;
        for (auto p : m_global_path){
            points.push_back(p.position);
        }

        m_global_markers.markers[0].header.stamp = ros::Time::now();
        m_global_markers.markers[0].points = points;

        // global path yaw
        for (int i = 0; i < m_global_path.size(); i++){
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
            yaw.pose = m_global_path[i];
            yaw.color = RED;
            yaw.color.a = 0.4f;
            yaw.lifetime = ros::Duration();
            m_global_markers.markers.push_back(yaw);
        }

        prev_mission = mission;
    }
    else{
        m_global_marker_pub.publish(m_global_markers);
    }
}
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "global_path_visual");
    kuam::Visualizer visualizer;

    ros::spin();

    return 0;
}