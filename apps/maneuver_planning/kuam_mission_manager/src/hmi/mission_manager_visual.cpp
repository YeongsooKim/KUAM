#include <ros/ros.h>
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/format.hpp>

#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/TransReq.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_rviz_plugins/OverlayText.h>

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
    ros::Subscriber m_tasklist_sub;

    // Publisher
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

    string m_tasklist;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    void TaskListCallback(const kuam_msgs::TaskList::ConstPtr &ego_ptr);
    void TextPubCallback(const ros::TimerEvent& event);
};


Visualizer::Visualizer() :
    m_p_nh("~"),
    m_process_freq_param(NAN),
    m_text_width_param(NAN),
    m_text_height_param(NAN),
    m_text_left_param(NAN),
    m_text_top_param(NAN)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[mission_manager_visual] Fail GetParam %s", m_err_param.c_str());
    InitROS();
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
    m_tasklist_sub = 
        m_nh.subscribe<kuam_msgs::TaskList>("mission_manager/tasklist", 10, boost::bind(&Visualizer::TaskListCallback, this, _1));

    // Initialize publisher
    m_text_pub = m_p_nh.advertise<jsk_rviz_plugins::OverlayText>("text", 10);
    
    // Initialize timer
    m_text_pub_cb_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Visualizer::TextPubCallback, this);

    return true;
}

void Visualizer::TaskListCallback(const kuam_msgs::TaskList::ConstPtr &tasklist_ptr)
{
    string lb = "\n";
    string tab = "\t";
    stringstream ss;

    auto task_list = tasklist_ptr->task;
    auto status_list = tasklist_ptr->status;

    for (int i = 0; i < task_list.size(); i++){
        ss << task_list[i] << ":\t" << status_list[i] << lb;
    }
    m_tasklist = ss.str();
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
    auto tasklist = m_tasklist;
    
    text.text = "Task list: \n" + tasklist;
    text.fg_color = fg_color;
    text.bg_color = bg_color;

    m_text_pub.publish(text);
}
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "mission_manager_visual");
    kuam::Visualizer kuam_visualizer;

    ros::spin();

    return 0;
}
