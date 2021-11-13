#include <ros/ros.h>
#include <vector>

#include <tf2/LinearMath/Quaternion.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <kuam_msgs/VehicleState.h>

using namespace std;

namespace kuam{

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
    ros::Subscriber m_detected_vehicle_position_sub;

    // Publisher
    ros::Publisher m_vehicle_ground_truth_pub;
    ros::Publisher m_detected_vehicle_pub;

    // Timer
    ros::Timer m_marker_pub;

    // Param
    string m_err_param;
    float m_process_freq_param;
    float m_target_pos_x_param;
    float m_target_pos_y_param;
    float m_target_pos_z_param;
    float m_target_euler_r_param;
    float m_target_euler_p_param;
    float m_target_euler_y_param;

    // Flag

    // Marker
    visualization_msgs::Marker m_ground_truth_vehicle_marker;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitMarkers();

    void MarkerPubCallback(const ros::TimerEvent& event);
    void VehicleStateCallback(const kuam_msgs::VehicleState::ConstPtr vehicle_state_ptr);
};


Visualizer::Visualizer() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[vehicle_position_visual] Fail GetParam %s", m_err_param.c_str());
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
    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("target_pos_x", m_target_pos_x_param)) { m_err_param = "target_pos_x"; return false; }
    if (!m_p_nh.getParam("target_pos_y", m_target_pos_y_param)) { m_err_param = "target_pos_y"; return false; }
    if (!m_p_nh.getParam("target_pos_z", m_target_pos_z_param)) { m_err_param = "target_pos_z"; return false; }
    if (!m_p_nh.getParam("target_euler_r", m_target_euler_r_param)) { m_err_param = "target_euler_r"; return false; }
    if (!m_p_nh.getParam("target_euler_p", m_target_euler_p_param)) { m_err_param = "target_euler_p"; return false; }
    if (!m_p_nh.getParam("target_euler_y", m_target_euler_y_param)) { m_err_param = "target_euler_y"; return false; }

    return true;
}

bool Visualizer::InitROS()
{
    // Initialize subscriber
    m_detected_vehicle_position_sub = 
        m_nh.subscribe<kuam_msgs::VehicleState>("vehicle_position/vehicle_state", 10, boost::bind(&Visualizer::VehicleStateCallback, this, _1));

    // Initialize publisher
    m_vehicle_ground_truth_pub = m_p_nh.advertise<visualization_msgs::Marker>("ground_truth_vehicle", 10);
    m_detected_vehicle_pub = m_p_nh.advertise<visualization_msgs::Marker>("detected_vehicle", 10);
    
    // Initialize timer
    m_marker_pub = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Visualizer::MarkerPubCallback, this);

    return true;
}

bool Visualizer::InitMarkers()
{
    std_msgs::ColorRGBA GRAY;
    GRAY.r = 204.0/255.0; GRAY.g = 204.0/255.0; GRAY.b = 204.0/255.0;

    tf2::Quaternion q;
    q.setRPY(m_target_euler_r_param, m_target_euler_p_param, m_target_euler_y_param+1.57);
    
    //// Ground truth target vehicle
    m_ground_truth_vehicle_marker.header.frame_id = "map";
    m_ground_truth_vehicle_marker.ns = "ground_truth/vehicle";
    m_ground_truth_vehicle_marker.id = 0;
    m_ground_truth_vehicle_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    m_ground_truth_vehicle_marker.mesh_resource = "package://kuam_vehicle_position/rviz/models/nissan_skyline_250gt/nissan_skyline_250gt.stl";
    m_ground_truth_vehicle_marker.action = visualization_msgs::Marker::ADD;
    m_ground_truth_vehicle_marker.scale.x = 0.26;
    m_ground_truth_vehicle_marker.scale.y = 0.26;
    m_ground_truth_vehicle_marker.scale.z = 0.26;
    m_ground_truth_vehicle_marker.pose.position.x = m_target_pos_x_param;
    m_ground_truth_vehicle_marker.pose.position.y = m_target_pos_y_param;
    m_ground_truth_vehicle_marker.pose.position.z = m_target_pos_z_param;
    m_ground_truth_vehicle_marker.pose.orientation.x = q.x();
    m_ground_truth_vehicle_marker.pose.orientation.y = q.y();
    m_ground_truth_vehicle_marker.pose.orientation.z = q.z();
    m_ground_truth_vehicle_marker.pose.orientation.w = q.w();
    m_ground_truth_vehicle_marker.color = GRAY;
    m_ground_truth_vehicle_marker.color.a = 0.8f;
    m_ground_truth_vehicle_marker.lifetime = ros::Duration();
}

void Visualizer::MarkerPubCallback(const ros::TimerEvent& event)
{
    m_ground_truth_vehicle_marker.header.stamp = ros::Time::now();
    m_vehicle_ground_truth_pub.publish(m_ground_truth_vehicle_marker);
}

void Visualizer::VehicleStateCallback(const kuam_msgs::VehicleState::ConstPtr vehicle_state_ptr)
{
    std_msgs::ColorRGBA GREEN;
    GREEN.r = 051.0/255.0; GREEN.g = 255.0/255.0; GREEN.b = 051/255.0;

    visualization_msgs::Marker detected_vehicle_marker;
    detected_vehicle_marker.header.frame_id = vehicle_state_ptr->header.frame_id;
    detected_vehicle_marker.header.stamp = ros::Time::now();
    detected_vehicle_marker.ns = "detected_vehicle";
    detected_vehicle_marker.id = 0;
    detected_vehicle_marker.type = visualization_msgs::Marker::SPHERE;
    detected_vehicle_marker.action = visualization_msgs::Marker::ADD;
    detected_vehicle_marker.scale.x = 0.3;
    detected_vehicle_marker.scale.y = 0.3;
    detected_vehicle_marker.scale.z = 0.3;
    detected_vehicle_marker.pose = vehicle_state_ptr->vehicle;
    detected_vehicle_marker.color = GREEN;
    detected_vehicle_marker.color.a = 1.0f;
    detected_vehicle_marker.lifetime = ros::Duration(0.1);

    m_detected_vehicle_pub.publish(detected_vehicle_marker);
}

}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "vehicle_position_visual");
    kuam::Visualizer kuam_visualizer;

    ros::spin();

    return 0;
}
