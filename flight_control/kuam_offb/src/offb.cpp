#include "kuam_offb/offb.h"
#include <string>

#include <geometry_msgs/Point.h>
#include <mavros_msgs/PositionTarget.h>

using namespace std;
using namespace kuam;
using namespace mission;
    

Offboard::Offboard() : 
    m_tfListener(m_tfBuffer),
    m_last_request_time(ros::Time::now()),
    m_process_freq_param(NAN),
    m_maneuver_ns_param("missing"),
    m_prev_sm_state("none")
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[offb] Fail GetParam");
    InitROS();
    if (!InitClient()) ROS_ERROR_STREAM("[offb] Fail InitClient");
}

Offboard::~Offboard()
{}

bool Offboard::InitFlag()
{
    m_is_global = false;

    return true;
}

bool Offboard::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/process_freq", m_process_freq_param);
    m_nh.getParam(nd_name + "/maneuver_ns", m_maneuver_ns_param);
    m_nh.getParam(nd_name + "/maneuver_ns", m_maneuver_ns_param);

    if (__isnan(m_process_freq_param)) { ROS_ERROR_STREAM("m_process_freq_param is NAN"); return false; }
    else if (m_maneuver_ns_param == "missing") { ROS_ERROR_STREAM("m_maneuver_ns_param is missing"); return false; }

    return true;
}

bool Offboard::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();

    // Initialize subscriber
    m_mavros_state_sub = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&Offboard::MavrosStateCallback, this, _1));
    m_setpoint_sub = m_nh.subscribe<kuam_msgs::Setpoint>(m_maneuver_ns_param + "/state_machine/setpoint", 10, boost::bind(&Offboard::SetpointCallback, this, _1));
    m_trans_req_sub = m_nh.subscribe<kuam_msgs::TransReq>(m_maneuver_ns_param + "/state_machine/trans_request", 10, boost::bind(&Offboard::TransReqCallback, this, _1));

    // Initialize publisher
    m_local_pos_tar_pub = m_nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    m_offboard_state_pub = m_nh.advertise<uav_msgs::OffboardState>(nd_name + "/offboard_state", 1000);

    // Initialize service client
    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(1.0/m_process_freq_param), &Offboard::ProcessTimerCallback, this);

    return true;
}

bool Offboard::InitClient()
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

void Offboard::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_sm_kuam_mode == "MANUAL"){
        ManualRequest();
    }
    else if (m_sm_kuam_mode == "OFFBOARD"){
        if (IsOffboard()){
            switch ((int)String2State(m_cur_sm_state)){
                case (int)State::Standby: m_offb_state = Enum2String(State::Standby); break;
                case (int)State::Takeoff: TakeoffRequest(); break;
                case (int)State::Arm: ArmRequest(); break;
                case (int)State::Hovering: HoveringRequest(); break;
                case (int)State::Docking: break;
                case (int)State::Undocking: break;
                case (int)State::Flight: FlightRequest(); break;
                case (int)State::Transition: break;
                case (int)State::Landing: LandingRequest(); break;
                default: ROS_ERROR_STREAM("not in case"); break;
            }
            StateUpdate();
        }
    }
    else if (m_sm_kuam_mode == "EMERG"){
        EmergRequest();
    }
    
    SetpointPub();
    OffbStatusPub();
}

void Offboard::ManualRequest()
{
    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    if( m_px4_mode != "MANUAL" && 
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(manual_set_mode) &&
            manual_set_mode.response.mode_sent){
            // m_offb_state = Enum2String(Mode::Manual);
            ROS_INFO("Manual enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

void Offboard::EmergRequest()
{
    mavros_msgs::SetMode emerg_mode;
    emerg_mode.request.custom_mode = "AUTO.RTL";

    if( m_px4_mode != "AUTO.RTL" && 
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(emerg_mode) &&
            emerg_mode.response.mode_sent){

            ROS_INFO("Emergy enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

bool Offboard::IsOffboard()
{
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if( m_px4_mode != "OFFBOARD" &&
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
        }
        m_last_request_time = ros::Time::now();
        return false;
    }
    return true;
}

void Offboard::ArmRequest()
{
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if(!m_mavros_status.armed &&
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if( m_arming_serv_client.call(arm_cmd) &&
            arm_cmd.response.success){

            m_offb_state = Enum2String(Trans::Arm);
            ROS_INFO("Vehicle armed");
        }
        m_last_request_time = ros::Time::now();
    }
}

void Offboard::HoveringRequest()
{
    if (m_prev_sm_state == Enum2String(State::Takeoff)) m_offb_state = "TakeoffHovering";
    else if (m_prev_sm_state == Enum2String(State::Flight)) m_offb_state = "FlightHovering";
}


void Offboard::TakeoffRequest()
{
    m_offb_state = Enum2String(State::Takeoff);
}

void Offboard::LandingRequest()
{
    if (m_is_land){
        mavros_msgs::SetMode landing_mode;
        landing_mode.request.custom_mode = "AUTO.LAND";

        if (m_mavros_status.mode != "AUTO.LAND" && m_mavros_status.armed && 
            (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
            if(m_set_mode_serv_client.call(landing_mode) &&
                landing_mode.response.mode_sent){

                ROS_INFO("Land enabled");
            }
            m_last_request_time = ros::Time::now();
        }
        else if (!m_mavros_status.armed){
            m_offb_state = Enum2String(Trans::Landing);
        }
    }
}

void Offboard::StateUpdate()
{ 
    if (m_prev_sm_state != m_cur_sm_state){
        m_prev_sm_state = Ascii2Lower(m_cur_sm_state);
    }
}

void Offboard::FlightRequest()
{
    m_offb_state = Enum2String(State::Flight);
}


void Offboard::SetpointPub()
{
    mavros_msgs::PositionTarget pos_tar;
    pos_tar.header.stamp = ros::Time::now();
    pos_tar.header.frame_id = "map";

    pos_tar.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX
        | mavros_msgs::PositionTarget::IGNORE_AFY
        | mavros_msgs::PositionTarget::IGNORE_AFZ
        | mavros_msgs::PositionTarget::FORCE
        | mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // This makes it so our velocities are sent in the UAV frame.
    // If you use FRAME_LOCAL_NED then it will be in map frame
    pos_tar.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // Set the velocities we want (it is in m/s)
    // On the real drone send a minimum of 0.3 m/s in x/y (except when you send 0.0 m/s).
    // If the velocity is lower than 0.3 m/s then the UAV can move in any direction!

    pos_tar.position = m_setpoint_pose.position;
    pos_tar.velocity.x = m_setpoint_vel.linear.x;
    pos_tar.velocity.y = m_setpoint_vel.linear.y;
    pos_tar.velocity.z = m_setpoint_vel.linear.z;

    auto euler = m_utils.Quat2Euler(m_setpoint_pose.orientation);
    auto yaw_rad = euler.y;
    if (__isnan(yaw_rad)) yaw_rad = 0.0;

    pos_tar.yaw = yaw_rad;
    
    // Set the yaw rate (it is in rad/s)
    // pos_tar.yaw_rate = 0.1;
    
    m_local_pos_tar_pub.publish(pos_tar);
}

void Offboard::OffbStatusPub()
{
    uav_msgs::OffboardState offboard_status_msg;
    offboard_status_msg.offb_mode = m_mavros_status.mode;
    offboard_status_msg.offb_state = m_offb_state;
    m_offboard_state_pub.publish(offboard_status_msg);
}