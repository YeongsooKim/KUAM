#ifndef __MISSION_MANAGER__
#define __MISSION_MANAGER__

// ros
#include <ros/ros.h>
#include <uav_msgs/Chat.h>
#include <kuam_state_machine/state_machine.h>

// message
#include <std_msgs/String.h>
#include <kuam_msgs/Completion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geographic_msgs/GeoPose.h>
#include <geographic_msgs/GeoPoint.h>
#include <geographic_msgs/GeoPath.h>
#include <kuam_msgs/Waypoints.h>
#include <mavros_msgs/HomePosition.h>
#include <kuam_msgs/GlobalPathSync.h>

using namespace std;

namespace kuam{

using namespace mission;
const int NO_TASK = 99;
using trans_to_geoposes_pair = pair<Trans, vector < geographic_msgs::GeoPose > >;
using task_to_status_pair = pair<trans_to_geoposes_pair, Status>;
using id_to_taskStatus_map = map <int, task_to_status_pair>;

class Maneuver
{
    struct GlobalPath{
        int id;
        kuam_msgs::Waypoints waypoint;
    };

private:
    // Node Handler
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;

    id_to_taskStatus_map m_id_to_taskStatus_list;
    GlobalPath m_waypoints;
    
public:
    Maneuver();
    virtual ~Maneuver();

private:
    // Subscriber
    ros::Subscriber m_command_sub;
    ros::Subscriber m_completion_sub;
    ros::Subscriber m_kuam_waypoints_sub;

    // Publisher
    ros::Publisher m_task_pub;
    ros::Publisher m_tasklist_pub;

    // Service
    ros::ServiceServer m_global_path_sync_srv;

    // Timer
    ros::Timer m_mission_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    string m_data_ns_param;

    // Flag
    int m_cur_task_id;
    kuam_msgs::Completion m_completion;

private: // Function
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    
    void ProcessTimerCallback(const ros::TimerEvent& event);

    void ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr);
    void WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr);
    inline void CompletionCallback(const kuam_msgs::Completion::ConstPtr &msg_ptr) { m_completion = *msg_ptr; }
    bool GlobalPathMsgSync(kuam_msgs::GlobalPathSync::Request &req, kuam_msgs::GlobalPathSync::Response &res);

    bool IsTransition(string input);    // Check whether the transition is included in enum class
    bool InsertTaskStatus(string input, vector<geographic_msgs::GeoPose> geoposes=vector<geographic_msgs::GeoPose>());
    bool IsTaskRunning();
    bool HasTodoTask();
    bool DoTask();
    void CheckComplete();

    void TaskListPub();
};
}
#endif //  __MISSION_MANAGER__