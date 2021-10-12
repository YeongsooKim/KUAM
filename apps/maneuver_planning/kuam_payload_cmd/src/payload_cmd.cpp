#include "kuam_payload_cmd/payload_cmd.h"
#include <string>

#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;
using namespace kuam;
using namespace mission;
    

Playload::Playload() : 
    m_p_nh("~"),
    m_tfListener(m_tfBuffer),
    m_last_request_time(ros::Time::now()),
    m_chat_cmd_mode("none")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR("[payload_cmd] Fail GetParam %s", m_err_param.c_str());
    InitROS();
    if (!InitClient()) ROS_ERROR_STREAM("[payload_cmd] Fail InitClient");
}

Playload::~Playload()
{}

bool Playload::InitFlag()
{
    m_setpoint.is_global = false;

    return true;
}

bool Playload::GetParam()
{
    string nd_name = ros::this_node::getName();

    if (!m_p_nh.getParam("process_freq", m_process_freq_param)) { m_err_param = "process_freq"; return false; }
    if (!m_p_nh.getParam("data_ns", m_data_ns_param)) { m_err_param = "data_ns"; return false; }

    return true;
}

bool Playload::InitROS()
{
    // Initialize subscriber
    m_mavros_state_sub = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&Playload::MavrosStateCallback, this, _1));
    m_setpoint_sub = m_nh.subscribe<kuam_msgs::Setpoint>("state_machine/setpoint", 10, boost::bind(&Playload::SetpointCallback, this, _1));
    m_command_sub = m_nh.subscribe<uav_msgs::Chat>(m_data_ns_param + "/chat/command", 10, boost::bind(&Playload::ChatterCallback, this, _1));

    // Initialize publisher
    m_global_pose_pub = m_nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    m_local_pos_tar_pub = m_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    m_payload_cmd_pub = m_p_nh.advertise<uav_msgs::PayloadCmd>("payload_cmd", 1000);
    
    // Initialize service client
    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    m_landing_request_service = m_p_nh.advertiseService("land_request", &Playload::LandRequestSrv, this);
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Playload::ProcessTimerCallback, this);

    return true;
}

bool Playload::InitClient()
{
    if (m_process_freq_param == 0.0) return false;
    ros::Rate rate(m_process_freq_param);

    // wait for FCU
    while(ros::ok() && !m_mavros_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}


void Playload::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_chat_cmd_mode == "manual"){
        m_chat_cmd_mode = "none";
        ManualRequest();
    }
    else if (m_chat_cmd_mode == "altctl"){
        m_chat_cmd_mode = "none";
        AltitudeRequest();
    }
    else if (m_chat_cmd_mode == "offboard"){
        m_chat_cmd_mode = "none";
        OffboardRequest();
    }
    else if (m_chat_cmd_mode == "emergy"){
        m_chat_cmd_mode = "none";
        EmergRequest();
    }
    else if (m_chat_cmd_mode == "land"){
        m_chat_cmd_mode = "none";
        LandRequest();
    }

    if (m_chat_cmd_mode == "arm"){
        m_chat_cmd_mode = "none";
        ArmRequest();
    }
    
    SetpointPub();
    PayloadCmdPub();
}

void Playload::ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr)
{
    m_chat_cmd_mode = chat_ptr->msg;
    string msg = Ascii2Lower(chat_ptr->msg);
    if (IsMode(msg)){
        m_chat_cmd_mode = msg;
    }
}

bool Playload::LandRequestSrv(kuam_msgs::LandRequest::Request &req, kuam_msgs::LandRequest::Response &res)
{
    if (req.land_request){
        ROS_WARN_STREAM("[payload_cmd] Enter LandRequestSrv");
        LandRequest();

        ros::Rate rate(m_process_freq_param);
        while(m_mavros_status.armed){

            ros::spinOnce();
            rate.sleep();
        }
        ROS_WARN_STREAM("[payload_cmd] Complete LandRequestSrv");
        res.is_complete = true;
    }

    return true;
}


void Playload::ArmRequest()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if(!m_mavros_status.armed){
        if( m_arming_serv_client.call(arm_cmd) && arm_cmd.response.success){

            ROS_WARN("[payload_cmd] Vehicle armed");
        }
        m_last_request_time = ros::Time::now();
    }
}

void Playload::ManualRequest()
{
    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    if( m_mavros_status.mode != "MANUAL"){
        if(m_set_mode_serv_client.call(manual_set_mode) &&
            manual_set_mode.response.mode_sent){
            ROS_WARN("[payload_cmd] Manual enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

void Playload::AltitudeRequest()
{
    mavros_msgs::SetMode altitude_set_mode;
    altitude_set_mode.request.custom_mode = "ALTCTL";

    if( m_mavros_status.mode != "ALTCTL"){
        if(m_set_mode_serv_client.call(altitude_set_mode) &&
            altitude_set_mode.response.mode_sent){
            ROS_WARN("[payload_cmd] Altitude enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

bool Playload::OffboardRequest()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if( m_mavros_status.mode != "OFFBOARD"){
        if(m_set_mode_serv_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_WARN("[payload_cmd] Offboard enabled");
        }
        m_last_request_time = ros::Time::now();
        return false;
    }
    return true;
}

void Playload::LandRequest()
{
    mavros_msgs::SetMode landing_mode;
    landing_mode.request.custom_mode = "AUTO.LAND";

    if (m_mavros_status.mode != "AUTO.LAND" && m_mavros_status.armed){
        if(m_set_mode_serv_client.call(landing_mode) &&
            landing_mode.response.mode_sent){
            ROS_WARN("[payload_cmd] Land enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

void Playload::EmergRequest()
{
    mavros_msgs::SetMode emerg_mode;
    emerg_mode.request.custom_mode = "AUTO.RTL";

    if( m_mavros_status.mode != "AUTO.RTL"){
        if(m_set_mode_serv_client.call(emerg_mode) &&
            emerg_mode.response.mode_sent){

            ROS_ERROR("[payload_cmd] Emergy enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}


void Playload::SetpointPub()
{
    if (m_setpoint.is_global){
        geographic_msgs::GeoPoseStamped msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();

        msg.header = m_setpoint.header;
        msg.pose = m_setpoint.geopose;

        m_global_pose_pub.publish(msg);
    }
    else {
        mavros_msgs::PositionTarget pos_tar;
        pos_tar.header.stamp = ros::Time::now();
        pos_tar.header.frame_id = "base_link";

        pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_PX
            | mavros_msgs::PositionTarget::IGNORE_PY
            | mavros_msgs::PositionTarget::IGNORE_PZ 
            | mavros_msgs::PositionTarget::IGNORE_AFX
            | mavros_msgs::PositionTarget::IGNORE_AFY
            | mavros_msgs::PositionTarget::IGNORE_AFZ
            | mavros_msgs::PositionTarget::FORCE
            | mavros_msgs::PositionTarget::IGNORE_YAW;

        pos_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;

        pos_tar.velocity.x = m_setpoint.vel.linear.x;
        pos_tar.velocity.y = m_setpoint.vel.linear.y;
        pos_tar.velocity.z = m_setpoint.vel.linear.z;

        auto euler = m_utils.Quat2Euler(m_setpoint.yaw_rate.orientation);
        auto yaw_rate_rads = euler.y;
        if (__isnan(yaw_rate_rads)){ yaw_rate_rads = 0.0; }
        pos_tar.yaw_rate = yaw_rate_rads;
        
        m_local_pos_tar_pub.publish(pos_tar);
    }
}

void Playload::PayloadCmdPub()
{
    static string prev_mode = "none";
    if (prev_mode != m_mavros_status.mode){
        uav_msgs::PayloadCmd payload_cmd_msg;
        payload_cmd_msg.mode = m_mavros_status.mode;
        m_payload_cmd_pub.publish(payload_cmd_msg);

        prev_mode = m_mavros_status.mode;
    }
}

bool Playload::IsMode(string input)
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