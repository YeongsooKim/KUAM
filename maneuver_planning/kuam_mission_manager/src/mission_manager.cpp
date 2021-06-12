#include "kuam_mission_manager/mission_manager.h"
#include "kuam_mission_manager/state_machine.h"

// message
#include <kuam_msgs/TaskList.h>
#include <kuam_msgs/Task.h>
#include <kuam_msgs/Mode.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace kuam
{
using namespace mission;

Maneuver::Maneuver() :
    m_process_freq_param(NAN),
    m_cur_task(NO_TASK),
    m_kuam_mode("none"),
    m_px4_mode("none"),
    m_data_ns_param("missing"),
    m_control_ns_param("missing")
{
    GetParam();
    InitFlag();
    InitROS();   

    m_offb_state.offb_mode = "none";
}

Maneuver::~Maneuver()
{}

bool Maneuver::InitFlag()
{
    m_is_init_auto_mission = false;
    m_is_home_set = false;
    m_has_cmd = false;

    return true;
}

bool Maneuver::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);
    m_nh.getParam(nd_name + "/mode", m_is_auto_param);
    m_nh.getParam(nd_name + "/data_ns", m_data_ns_param);
    m_nh.getParam(nd_name + "/control_ns", m_control_ns_param);
    m_nh.getParam(nd_name + "/home_poses", m_control_ns_param);

    if (m_data_ns_param == "missing") { ROS_ERROR_STREAM("[mission_manager] m_data_ns_param is missing"); return false; }
    else if (m_control_ns_param == "missing") { ROS_ERROR_STREAM("[mission_manager] m_control_ns_param is missing"); return false; }
    else if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("[mission_manager] m_process_freq_param is NAN"); return false; }
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
    m_mode_pub = m_nh.advertise<kuam_msgs::Mode>(nd_name + "/mode", 10);

    // // Initialize timer
    m_mission_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Maneuver::ProcessTimerCallback, this);
    
    return true;
}

void Maneuver::ProcessTimerCallback(const ros::TimerEvent& event)
{
    CheckModeChange();
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
    TaskListPub();
}

void Maneuver::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    string msg = chat_ptr->msg;

    if (IsTransition(msg)) InsertTask(msg);
    else if (IsMode(msg)){
        if (msg != m_kuam_mode){
            m_cmd_mode = msg;
            m_has_cmd = true;
        }
    } 
}

void Maneuver::WaypointsCallback(const kuam_msgs::Waypoints::ConstPtr &wps_ptr)
{
    if (!m_is_init_auto_mission){
        m_is_init_auto_mission = true;
        
        // Get missions
        vector<string> missions;
        for (auto wp : wps_ptr->waypoints){
            string mission = wp.mission;
            missions.push_back(mission);
        }

        // Get pose
        vector<geometry_msgs::Pose> poses;
        vector<geographic_msgs::GeoPose> geoposes;
        for (auto wp : wps_ptr->waypoints){
            geoposes.push_back(wp.geopose);

            auto lat = wp.geopose.position.latitude;
            auto lon = wp.geopose.position.longitude;
            auto alt = wp.geopose.position.altitude;
            auto pose = m_utils.ConvertToMapFrame(lat, lon, alt, m_home_position);
            poses.push_back(pose);
        }

        // Allocate orientation
        for (int i = 0; i < poses.size(); i++){
            if (poses.size() != 1){
                if (i + 1 < poses.size()){
                    auto orientation = m_utils.GetOrientation(poses[i].position, poses[i+1].position);
                    poses[i].orientation = orientation;
                    geoposes[i].orientation = orientation;
                }
                else {
                    poses[i].orientation = poses[i-1].orientation;
                    geoposes[i].orientation = geoposes[i-1].orientation;
                }
            }
            else{
                poses[i].orientation.x = 0.0;
                poses[i].orientation.y = 0.0;
                poses[i].orientation.z = 0.0;
                poses[i].orientation.w = 1.0;

                geoposes[i].orientation.x = 0.0;
                geoposes[i].orientation.y = 0.0;
                geoposes[i].orientation.z = 0.0;
                geoposes[i].orientation.w = 1.0;
            }
        }

        // Allocate mission to posearray
        mssn_geoposes_map mssn_wps;
        string str = "none";
        for (auto mission : missions){
            if (str != mission){
                str = mission;
                geographic_msgs::GeoPath dummy;
                mssn_wps.insert(pair<string, geographic_msgs::GeoPath>(str, dummy));

                if (IsTransition(str)){
                    InsertTask(str);
                }
            }
        }

        // Allocate posearray
        for (auto& mssn_wp : mssn_wps){
            for (int i = 0; i < wps_ptr->waypoints.size(); i++){
                if (mssn_wp.first == wps_ptr->waypoints[i].mission){
                    geographic_msgs::GeoPoseStamped p;
                    p.header.frame_id = "map";
                    p.header.stamp = ros::Time::now();
                    p.pose = geoposes[i];
                    mssn_wp.second.poses.push_back(p);
                }
            }
        }

        // Add flight last waypoint
        auto mssn_wps_it = mssn_wps.find("landing");
        if (mssn_wps_it != mssn_wps.end()){
            if (mssn_wps_it->second.poses.size() != 0){
                auto ld_pose = mssn_wps_it->second.poses.back();

                auto fl_poses_it = mssn_wps.find("flight");
                if (fl_poses_it != mssn_wps.end()){
                    fl_poses_it->second.poses.push_back(ld_pose);
                }
            }
        }

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

    if (input == "landing"){
        trans = Trans::Disarm;
        status = Status::Todo;

        task = make_pair(trans, status);
        m_tasklist.push_back(task);
    }
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
        geographic_msgs::GeoPath geopath;
        pose_array.header.frame_id = "map";
        if ((task_msg.task != "arm") && (task_msg.task != "disarm")){

            for (auto p : m_mssn_wps_map.find(task_msg.task)->second.poses){
                geographic_msgs::GeoPoseStamped geopose = p;

                geopath.poses.push_back(geopose);
            }
        }
        task_msg.geopath = geopath;

        m_task_pub.publish(task_msg);
    }
}

void Maneuver::CheckComplete()
{
    Task &task = m_tasklist[m_cur_task];

    switch((int)task.first){
        case (int)Trans::Arm:
        case (int)Trans::Disarm:
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
        case (int)Trans::Landing:
            if (m_offb_state.offb_state == Enum2String(Trans::Disarm)){
                task.second = Status::Done;
            }
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

void Maneuver::CheckModeChange()
{
    string offb_mode = Ascii2Lower(m_offb_state.offb_mode);
    kuam_msgs::Mode mode;
    bool is_mode_changed = false;
    if (m_has_cmd){
        mode.px4 = m_offb_state.offb_mode;
        mode.kuam = m_cmd_mode;
        m_kuam_mode = offb_mode;
        m_has_cmd = false;

        is_mode_changed = true;
    }
    else if (m_kuam_mode != offb_mode){
        mode.px4 = m_offb_state.offb_mode;
        m_px4_mode = m_offb_state.offb_mode;

        if (IsMode(offb_mode)){
            mode.kuam = offb_mode;
            m_kuam_mode = offb_mode;
        }
        else {
            mode.kuam = m_kuam_mode;
        }
        is_mode_changed = true;
    }
    else if (m_px4_mode != m_offb_state.offb_mode){
        mode.kuam = m_kuam_mode;
        mode.px4 = m_offb_state.offb_mode;
        m_px4_mode = m_offb_state.offb_mode;

        is_mode_changed = true;
    }

    if (is_mode_changed){
        m_mode_pub.publish(mode);
    }
}

void Maneuver::TaskListPub()
{
    // Publish whole tasks
    kuam_msgs::TaskList tasklist_msg;
    for (auto task : m_tasklist){
        tasklist_msg.task.push_back(Enum2String(task.first));
        tasklist_msg.status.push_back(Enum2String(task.second));
    }
    m_tasklist_pub.publish(tasklist_msg);
}
}