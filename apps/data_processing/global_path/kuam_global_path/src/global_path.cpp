#include <kuam_global_path/global_path.h>

using namespace std;

namespace kuam{
    
GlobalPath::GlobalPath() :
    m_p_nh("~")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[global_path] Fail GetParam %s", m_err_param.c_str());
    InitROS();
}

GlobalPath::~GlobalPath()
{
}

bool GlobalPath::InitFlag()
{
    m_is_home_set = false;
    return true;
}

bool GlobalPath::GetParam()
{
    string ns_name = ros::this_node::getNamespace();

    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    else if (!m_p_nh.getParam("using_fake_gps", m_using_fake_gps_param)) { m_err_param = "using_fake_gps"; return false; }
    else if (!m_p_nh.getParam("m_latitude_param", m_latitude_param)) { m_err_param = "latitude"; return false; }
    else if (!m_p_nh.getParam("m_longitude_param", m_longitude_param)) { m_err_param = "longitude"; return false; }
    else if (!m_p_nh.getParam("m_altitude_param", m_altitude_param)) { m_err_param = "altitude"; return false; }

    return true;
}

bool GlobalPath::InitROS()
{
    // package, node, topic name
    string ns_name = ros::this_node::getNamespace();

    // Initialize subscriber
    if (m_using_fake_gps_param){
        m_home_position.latitude = m_latitude_param;
        m_home_position.longitude = m_longitude_param;
        m_home_position.altitude = m_altitude_param;
    }
    else {
        m_home_position_sub = 
            m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&GlobalPath::HomePositionCallback, this, _1));

        ros::Rate rate(10);
        while (ros::ok() && !m_is_home_set){
            ros::spinOnce();
            rate.sleep();
        }
    }

    ROS_WARN("%f", m_home_position.latitude);
    ROS_WARN("%f", m_home_position.longitude);
    ROS_WARN("%f", m_home_position.altitude);
    
    // Initialize publisher
    // m_image_pub = m_p_nh.advertise<sensor_msgs::Image>("output_video", 10);
    
    // Initialize timer
    m_image_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &GlobalPath::ProcessTimerCallback, this);

    return true;
}

void GlobalPath::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[global_path] Home set");
    }
}

void GlobalPath::ProcessTimerCallback(const ros::TimerEvent& event)
{
}
}