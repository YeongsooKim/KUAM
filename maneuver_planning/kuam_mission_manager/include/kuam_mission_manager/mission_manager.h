#ifndef __MISSION_MANAGER__
#define __MISSION_MANAGER__

// ros
#include <ros/ros.h>
#include <uav_msgs/Chat.h>
#include <kuam_mission_manager/state_machine.h>

// message
#include <std_msgs/String.h>
#include <uav_msgs/PayloadCmd.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPath.h>
#include <kuam_msgs/Waypoints.h>
#include <mavros_msgs/HomePosition.h>

#include <kuam_mission_manager/utils.h>

using namespace std;

namespace kuam{

using namespace mission;
const int NO_TASK = 99;
using mssn_geoposes_map = map < string, geographic_msgs::GeoPath >;

class Maneuver
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    TaskList m_tasklist;
    Utils m_utils;
    
public:
    Maneuver();
    virtual ~Maneuver();

private:
    // Subscriber
    ros::Subscriber m_command_sub;
    ros::Subscriber m_complete_sub;
    ros::Subscriber m_kuam_waypoints_sub;
    ros::Subscriber m_aruco_state_sub;
    ros::Subscriber m_home_position_sub;

    // Publisher
    ros::Publisher m_task_pub;
    ros::Publisher m_tasklist_pub;
    ros::Publisher m_mode_pub;
    
    // Timer
    ros::Timer m_mission_timer;

    // Param
    float m_process_freq_param;
    bool m_is_auto_param; // Dictionary
    string m_data_ns_param;

    // Flag
    bool m_is_init_auto_mission;
    bool m_is_home_set;
    bool m_has_cmd;

    int m_cur_task;
    string m_kuam_mode;
    string m_px4_mode;
    string m_cmd_mode;
    mssn_geoposes_map m_mssn_wps_map;
    uav_msgs::PayloadCmd m_payload_cmd;
    geographic_msgs::GeoPoint m_home_position;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    void ProcessTimerCallback(const ros::TimerEvent& event);

    void ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr);
    void WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wp_ptr);
    inline void PlayloadCmdCallback(const uav_msgs::PayloadCmd::ConstPtr &cmd_ptr) { m_payload_cmd = *cmd_ptr; }
    void HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr);

    bool IsTransition(string input);    // Check whether the transition is included in enum class
    bool IsMode(string input);
    bool InsertTask(string input);
    bool IsTaskRunning();
    bool HasTodoTask();
    bool DoTask();
    void CheckComplete();
    void CheckModeChange();

    void TaskListPub();
};
}
#endif //  __MISSION_MANAGER__