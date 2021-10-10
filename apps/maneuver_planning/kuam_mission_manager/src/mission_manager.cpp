#include "kuam_mission_manager/mission_manager.h"
#include "kuam_state_machine/state_machine.h"

// message
#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/Task.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace kuam
{
using namespace mission;

Maneuver::Maneuver() :
    m_p_nh("~"),
    m_cur_task_id(NO_TASK)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[mission_manager] Fail GetParam %s", m_err_param.c_str());
    InitROS();   
}

Maneuver::~Maneuver()
{}

bool Maneuver::InitFlag()
{
    return true;
}

bool Maneuver::GetParam()
{
    string nd_name = ros::this_node::getName();

    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("data_ns", m_data_ns_param)) { m_err_param = "data_ns"; return false; }

    return true;
}

bool Maneuver::InitROS()
{
    // Initialize subscriber
    m_command_sub = 
        m_nh.subscribe<uav_msgs::Chat>(m_data_ns_param + "/chat/command", 10, boost::bind(&Maneuver::ChatterCallback, this, _1));
    m_kuam_waypoints_sub = 
        m_nh.subscribe<kuam_msgs::Waypoints>(m_data_ns_param + "/global_path/waypoints", 10, boost::bind(&Maneuver::WaypointsCallback, this, _1));
    m_completion_sub = 
        m_nh.subscribe<kuam_msgs::Completion>("state_machine/completion", 10, boost::bind(&Maneuver::CompletionCallback, this, _1));

    // Initialize publisher
    m_tasklist_pub = m_p_nh.advertise<kuam_msgs::TaskList>("tasklist", 10);
    m_task_pub = m_p_nh.advertise<kuam_msgs::Task>("task", 10);

    // Initialize service
    m_global_path_sync_srv = m_p_nh.advertiseService("global_path_msg_sync", &Maneuver::GlobalPathMsgSync, this);


    // Initialize timer
    m_mission_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Maneuver::ProcessTimerCallback, this);
    
    return true;
}

void Maneuver::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (!IsTaskRunning()){
        if (HasTodoTask()){
            DoTask();
        }
        else{
            m_cur_task_id = NO_TASK;
        }
    }
    else {
        CheckComplete();
    }
    TaskListPub();
}

void Maneuver::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    string msg = chat_ptr->msg;

    if (IsTransition(msg)) InsertTaskStatus(msg);
}

void Maneuver::WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr)
{
    if (wps_ptr->id != m_waypoints.id){
        m_waypoints.id = wps_ptr->id;

        // Alocate mission
        for (int i = 0; i < wps_ptr->waypoints.size(); i++){
            ROS_WARN("[mission_manager] mission: %s", wps_ptr->waypoints[i].mission.c_str());
            if (IsTransition(wps_ptr->waypoints[i].mission)){
                ROS_WARN("[mission_manager] InsertTaskStatus, geoposes size: %d", wps_ptr->waypoints[i].geoposes.size());
                InsertTaskStatus(wps_ptr->waypoints[i].mission, wps_ptr->waypoints[i].geoposes);
            }
        }
    }
}

bool Maneuver::GlobalPathMsgSync(kuam_msgs::GlobalPathSync::Request &req, kuam_msgs::GlobalPathSync::Response &res)
{
    if (!req.is_init){
        m_waypoints.id = 0;
        res.sync = true;
    }
    else{
        res.sync = false;
    }

    return true;
}

bool Maneuver::IsTransition(string input)
{
    string str = "invalid";
    for (int item = (int)Trans::Takeoff; item < (int)Trans::ItemNum; item++){
        Trans trans = static_cast<Trans>(item);

        string convert = Enum2String(trans);
        if (input == convert){
            str = convert;
            break;
        }
    }
    if (str != "invalid") return true;
    else return false;
}

bool Maneuver::InsertTaskStatus(string input, vector<geographic_msgs::GeoPose> geoposes)
{
    static unsigned int taskStatus_id = 0;
    Trans trans = String2Trans(input);
    trans_to_geoposes_pair trans_to_geoposes = make_pair(trans, geoposes);
    
    Status status = Status::Todo;
    task_to_status_pair task_to_status = make_pair(trans_to_geoposes, status);

    m_id_to_taskStatus_list.insert(pair<int, task_to_status_pair>(taskStatus_id, task_to_status));
    taskStatus_id++;
}

bool Maneuver::IsTaskRunning()
{
    for (auto id_to_taskStatus : m_id_to_taskStatus_list){
        auto taskStatus = id_to_taskStatus.second;
        if (taskStatus.second == Status::Doing){
            return true;
        }
    }
    return false;
}

bool Maneuver::HasTodoTask()
{
    int task_num = 0;
    for (auto id_to_taskStatus : m_id_to_taskStatus_list){
        auto taskStatus = id_to_taskStatus.second;
        if (taskStatus.second == Status::Todo){
            m_cur_task_id = id_to_taskStatus.first;

            return true;
        }
        task_num++;
    }
    return false;
}

bool Maneuver::DoTask()
{
    auto &taskStatus = m_id_to_taskStatus_list.find(m_cur_task_id)->second;
    auto &task = taskStatus.first;
    
    taskStatus.second = Status::Doing;

    kuam_msgs::Task task_msg;
    task_msg.task = Enum2String(task.first);
    
    geometry_msgs::PoseArray pose_array;
    geographic_msgs::GeoPath geopath;
    pose_array.header.frame_id = "map";
    if (!task.second.empty()){
        for (auto p : task.second){
            geographic_msgs::GeoPoseStamped g;
            g.pose = p;

            geopath.poses.push_back(g);
        }
    }

    task_msg.geopath = geopath;

    m_task_pub.publish(task_msg);
}

void Maneuver::CheckComplete()
{
    auto &taskStatus = m_id_to_taskStatus_list.find(m_cur_task_id)->second;
    auto &task = taskStatus.first;
    auto complete_task = String2Trans(m_completion.task);
    
    if (complete_task == task.first){
        if (m_completion.is_complete){
            m_completion.is_complete = false;
            taskStatus.second = Status::Done;
        }
    }
}

void Maneuver::TaskListPub()
{
    // Publish whole tasks
    kuam_msgs::TaskList tasklist_msg;
    for (auto id_to_taskStatus : m_id_to_taskStatus_list){
        auto taskStatus = id_to_taskStatus.second;
        auto task = taskStatus.first;

        tasklist_msg.task.push_back(Enum2String(task.first));
        tasklist_msg.status.push_back(Enum2String(taskStatus.second));
    }
    m_tasklist_pub.publish(tasklist_msg);
}
}