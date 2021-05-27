#include "kuam_mission_manager/mission_manager.h"
#include "kuam_mission_manager/state_machine.h"

// message
#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/Task.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace kuam
{
using namespace mission;

Maneuver::Maneuver() :
    m_cur_task(NO_TASK),
    m_mode("manual"),
    m_data_ns_param("missing"),
    m_control_ns_param("missing"),
    m_target_height_m_param(NAN)
{
    GetParam();
    InitFlag();
    InitROS();   
}

Maneuver::~Maneuver()
{}

bool Maneuver::InitFlag()
{
    m_is_init_auto_mission = false;
    m_is_home_set = false;

    return true;
}

bool Maneuver::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/mode", m_is_auto_param);
    m_nh.getParam(nd_name + "/data_ns", m_data_ns_param);
    m_nh.getParam(nd_name + "/control_ns", m_control_ns_param);
    m_nh.getParam(nd_name + "/home_poses", m_control_ns_param);
    m_nh.getParam(nd_name + "/target_height_m", m_target_height_m_param);

    if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("m_data_ns_param is missing"); return false; }
    else if (m_control_ns_param == "missing") { ROS_ERROR_STREAM("m_control_ns_param is missing"); return false; }
    else if (__isnan(m_target_height_m_param)) { ROS_ERROR_STREAM("m_target_height_m_param is NAN"); return false; }
    return true;
}

bool Maneuver::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();

    // Initialize subscriber
    m_home_position_sub = 
        m_nh.subscribe<mavros_msgs::HomePosition>("/mavros/home_position/home", 10, boost::bind(&Maneuver::HomePositionCallback, this, _1));

    ros::Rate rate(10);
    while (ros::ok() && !m_is_home_set){
        ros::spinOnce();
        rate.sleep();
    }

    m_command_sub = 
        m_nh.subscribe<uav_msgs::Chat>(m_data_ns_param + "/chat/command", 10, boost::bind(&Maneuver::ChatterCallback, this, _1));
    m_kuam_waypoints_sub = 
        m_nh.subscribe<kuam_msgs::Waypoints>(m_data_ns_param + "/csv_parser/waypoints", 10, boost::bind(&Maneuver::WaypointsCallback, this, _1));
    m_complete_sub = 
        m_nh.subscribe<uav_msgs::OffboardState>(m_control_ns_param + "/offb/offboard_state", 10, boost::bind(&Maneuver::OffbStateCallback, this, _1));

    // // Initialize publisher
    m_tasklist_pub = m_nh.advertise<kuam_msgs::TaskList>(nd_name + "/tasklist", 10);
    m_task_pub = m_nh.advertise<kuam_msgs::Task>(nd_name + "/task", 10);
    m_mode_pub = m_nh.advertise<std_msgs::String>(nd_name + "/mode", 10);
    m_waypoint_pub = m_nh.advertise<std_msgs::String>(nd_name + "/mode", 10);

    // // Initialize timer
    m_mission_timer = m_nh.createTimer(ros::Duration(0.05), &Maneuver::ProcessTimerCallback, this);
    
    return true;
}

void Maneuver::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (!IsTaskRunning()){
        if (HasTodoTask()){
            DoTask();
        }
        else{
            m_cur_task = NO_TASK;
        }
    }
    else {
        CheckComplete();
    }
    Publish();

}

void Maneuver::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    string msg = chat_ptr->msg;

    if (IsTransition(msg)) InsertTask(msg);
    else if (IsMode(msg)){
        m_mode = msg;

        // Publish current mode
        std_msgs::String mode_msg;
        mode_msg.data = m_mode;
        m_mode_pub.publish(mode_msg);
    } 
}

void Maneuver::WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr)
{
    if (!m_is_init_auto_mission){
        m_is_init_auto_mission = true;
        
        vector<string> missions;
        for (auto wp : wps_ptr->waypoints){
            string mission = wp.mission;
            missions.push_back(mission);
        }

        mssn_poses_map mssn_wps;
        string str = "none";
        for (auto mission : missions){
            if (str != mission){
                str = mission;
                poses dummy;
                mssn_wps.insert(pair<string, poses>(str, dummy));

                if (IsTransition(str)){
                    InsertTask(str);
                }
            }
        }

        for (auto& mssn_wp : mssn_wps){
            for (auto wp : wps_ptr->waypoints){
                if (mssn_wp.first == wp.mission){
                    auto lat = wp.geopose.position.latitude;
                    auto lon = wp.geopose.position.longitude;
                    auto alt = m_target_height_m_param;
                    auto pose = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
                    pose.orientation = wp.geopose.orientation;
                    mssn_wp.second.poses.push_back(pose);
                }
            }
        }

        // auto ld_pose = (mssn_wps.find("landing")->second).poses.back();
        // auto& fl_poses = mssn_wps.find("flight")->second;
        // fl_poses.poses.push_back(ld_pose);

        m_mssn_wps_map = mssn_wps;
    }
}

void Maneuver::HomePositionCallback(const mavros_msgs::HomePosition::ConstPtr &home_ptr)
{
    if (!m_is_home_set){
        m_is_home_set = true;

        m_home_position.latitude = home_ptr->geo.latitude;
        m_home_position.longitude = home_ptr->geo.longitude;
        m_home_position.altitude = home_ptr->geo.altitude;
        ROS_WARN_STREAM("[mission_manager] Home set");
    }
}

bool Maneuver::IsTransition(string input)
{
    string str = "invalid";
    for (int item = (int)Trans::Arm; item < (int)Trans::ItemNum; item++){
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

bool Maneuver::IsMode(string input)
{
    string str = "invalid";
    for (int item = (int)Mode::Manual; item < (int)Mode::ItemNum; item++){
        Mode mode = static_cast<Mode>(item);

        string convert = Enum2String(mode);
        if (input == convert){
            str = convert;
            break;
        }
    }
    if (str != "invalid") return true;
    else return false;
}

bool Maneuver::InsertTask(string input)
{
    Trans trans;
    Status status;
    Task task;

    if (input == "takeoff"){
        trans = Trans::Arm;
        status = Status::Todo;

        task = make_pair(trans, status);
        m_tasklist.push_back(task);    
    }

    trans = String2Trans(input);
    status = Status::Todo;

    task = make_pair(trans, status);
    m_tasklist.push_back(task);
}

bool Maneuver::IsTaskRunning()
{
    for (auto task : m_tasklist){
        if (task.second == Status::Doing){
            return true;
        }
    }
    return false;
}

bool Maneuver::HasTodoTask()
{
    int task_num = 0;
    for (auto task : m_tasklist){
        if (task.second == Status::Todo){
            m_cur_task = task_num;

            return true;
        }
        task_num++;
    }
    return false;
}

bool Maneuver::DoTask()
{
    m_tasklist[m_cur_task].second = Status::Doing;
    
    if (m_mssn_wps_map.empty()) {
        ROS_ERROR_STREAM ("[mission_manager] mssn_wps empty");

        return false;
    }
    else {
        kuam_msgs::Task task_msg;
        task_msg.task = Enum2String(m_tasklist[m_cur_task].first);

        geometry_msgs::PoseArray pose_array;
        pose_array.header.frame_id = "map";
        if ((task_msg.task != "arm") && (task_msg.task != "disarm")){

            for (auto p : m_mssn_wps_map.find(task_msg.task)->second.poses){
                geometry_msgs::Pose pose = p;

                pose_array.poses.push_back(pose);
            }
        }
        task_msg.pose_array = pose_array;

        m_task_pub.publish(task_msg);
    }
}

void Maneuver::CheckComplete()
{
    Task &task = m_tasklist[m_cur_task];

    switch((int)task.first){
        case (int)Trans::Arm:
        case (int)Trans::Landing:
            if (m_offb_state.offb_state == Enum2String(task.first)){
                task.second = Status::Done;
            }
            break;

        case (int)Trans::Takeoff:
            if (m_offb_state.offb_state == "TakeoffHovering"){
                task.second = Status::Done;
            }
            break;
        case (int)Trans::Flight:
            if (m_offb_state.offb_state == "FlightHovering"){
                task.second = Status::Done;
            }
            break;
        case (int)Trans::Disarm:
            
            break;

        case (int)Trans::Docking:
            
            break;

        case (int)Trans::Undocking:
            
            break;

        case (int)Trans::Transition:
            
            break;

        default: 
            break;
    }
}

void Maneuver::Publish()
{
    // Publish whole tasks
    kuam_msgs::TaskList tasklist_msg;
    vector<std_msgs::String> task_statuses;
    for (auto task : m_tasklist){
        stringstream msg;
        string trans = Enum2String(task.first);
        string status = Enum2String(task.second);
        
        msg << trans << "  |  " << status;
        
        std_msgs::String task_status;
        task_status.data = msg.str();

        tasklist_msg.task.push_back(task_status);
    }
    m_tasklist_pub.publish(tasklist_msg);
}
}