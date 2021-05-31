#include "kuam_offb/offb.h"
#include <string>

#include <geometry_msgs/Point.h>

using namespace std;
using namespace kuam;
using namespace mission;
    

geometry_msgs::Point marker_point;

Offboard::Offboard() : 
    m_tfListener(m_tfBuffer),
    m_last_request_time(ros::Time::now()),
    m_setpoint_pub_interval_param(NAN),
    m_maneuver_ns_param("missing"),
    m_prev_sm_state("none"),
    m_hovering_count(0)
{
    InitFlag();
    if (!GetParam()) ROS_ERROR_STREAM("[offb] Fail GetParam");
    InitROS();
    if (!InitClient()) ROS_ERROR_STREAM("[offb] Fail InitClient");

    m_setpoint.header.frame_id = "map";
}

Offboard::~Offboard()
{}

bool Offboard::InitFlag()
{
    m_is_global = false;
    m_offb_init = false;

    return true;
}

bool Offboard::GetParam()
{
    string nd_name = ros::this_node::getName();

    m_nh.getParam(nd_name + "/setpoint_pub_interval", m_setpoint_pub_interval_param);
    m_nh.getParam(nd_name + "/maneuver_ns", m_maneuver_ns_param);
    m_nh.getParam(nd_name + "/maneuver_ns", m_maneuver_ns_param);

    if (__isnan(m_setpoint_pub_interval_param)) { ROS_ERROR_STREAM("m_setpoint_pub_interval_param is NAN"); return false; }
    else if (m_maneuver_ns_param == "missing") { ROS_ERROR_STREAM("m_maneuver_ns_param is missing"); return false; }

    return true;
}

bool Offboard::InitROS()
{
    // package, node, topic name
    string nd_name = ros::this_node::getName();

    // Initialize subscriber
    m_mavros_state_sub = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&Offboard::MavrosStateCallback, this, _1));
    m_sm_state_sub = m_nh.subscribe<std_msgs::String>(m_maneuver_ns_param + "/state_machine/state", 10, boost::bind(&Offboard::SMStateCallback, this, _1));
    m_sm_mode_sub = m_nh.subscribe<std_msgs::String>(m_maneuver_ns_param + "/state_machine/mode", 10, boost::bind(&Offboard::SMModeCallback, this, _1));
    m_setpoint_sub = m_nh.subscribe<kuam_msgs::Setpoint>(m_maneuver_ns_param + "/state_machine/setpoint", 10, boost::bind(&Offboard::SetpointCallback, this, _1));
    m_landing_state_sub = m_nh.subscribe<kuam_msgs::LandingState>("/kuam/maneuver/state_machine/landing_state", 10, boost::bind(&Offboard::LandingStateCallback, this, _1));

    // Initialize publisher
    m_local_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    m_global_pose_pub = m_nh.advertise<geographic_msgs::GeoPoseStamped>("/mavros/setpoint_position/global", 10);
    m_velocity_pub = m_nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    m_offboard_state_pub = m_nh.advertise<uav_msgs::OffboardState>(nd_name + "/offboard_state", 1000);
    m_err_pub = m_nh.advertise<geometry_msgs::Point>(nd_name + "/error", 1000);

    // Initialize service client
    m_arming_serv_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    m_set_mode_serv_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    // Time callback
    m_timer = m_nh.createTimer(ros::Duration(m_setpoint_pub_interval_param), &Offboard::ProcessTimerCallback, this);

    return true;
}

bool Offboard::InitClient()
{
    if (m_setpoint_pub_interval_param == 0.0) return false;
    ros::Rate rate(1/m_setpoint_pub_interval_param);

    // wait for FCU
    while(ros::ok() && !m_mavros_status.connected){
        ros::spinOnce();
        rate.sleep();
    }

    return true;
}

void Offboard::ProcessTimerCallback(const ros::TimerEvent& event)
{
    if (m_sm_mode == "MANUAL"){
        m_offb_init = false;
        ManualRequest();
    }
    else if (m_sm_mode == "OFFBOARD"){
        if (IsOffboard()){
            switch ((int)String2State(m_sm_state)){
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
            SetpointnOffbStatePub(); // publish what?
            StateUpdate();
        }
    }
    else if (m_sm_mode == "EMERG"){
        m_offb_init = false;
    }
}

void Offboard::ManualRequest()
{
    mavros_msgs::SetMode manual_set_mode;
    manual_set_mode.request.custom_mode = "MANUAL";

    if( m_mavros_status.mode != "MANUAL" &&
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(manual_set_mode) &&
            manual_set_mode.response.mode_sent){

            m_offb_state = Enum2String(Mode::Manual);
            ROS_INFO("Manual enabled");
        }
        m_last_request_time = ros::Time::now();
    }
}

bool Offboard::IsOffboard()
{
    if (m_offb_init == false){
        ros::Rate rate(20.0);

        for(int i = 100; ros::ok() && i > 0; --i){
            SetpointnOffbStatePub();
            ros::spinOnce();
            rate.sleep();
        }
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    if( m_mavros_status.mode != "OFFBOARD" &&
        (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
        if(m_set_mode_serv_client.call(offb_set_mode) &&
            offb_set_mode.response.mode_sent){
            ROS_INFO("Offboard enabled");
            m_offb_init = true;
        }
        m_last_request_time = ros::Time::now();
        return false;
    }
    return true;
    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    // if( m_mavros_status.mode != "OFFBOARD" &&
    //     (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
    //     if(m_set_mode_serv_client.call(offb_set_mode) &&
    //         offb_set_mode.response.mode_sent){
    //         ROS_INFO("Offboard enabled");
    //     }
    //     m_last_request_time = ros::Time::now();
    // } else {
    //     if(!m_mavros_status.armed &&
    //         (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
    //         if( m_arming_serv_client.call(arm_cmd) &&
    //             arm_cmd.response.success){
    //             ROS_INFO("Vehicle armed");
    //         }
    //         m_last_request_time = ros::Time::now();
    //     }
    // }
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
    // m_offb_state = Enum2String(State::Takeoff);
}

void Offboard::LandingRequest()
{
    if (m_is_land){
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "AUTO.LAND";

        if (m_mavros_status.mode != "AUTO.LAND" && m_mavros_status.armed && 
            (ros::Time::now() - m_last_request_time > ros::Duration(5.0))){
            if(m_set_mode_serv_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){

                ROS_INFO("Land enabled");
            }
            m_last_request_time = ros::Time::now();
        }
        else if (!m_mavros_status.armed){
            m_offb_state = Enum2String(Trans::Landing);
        }
    }
    else if (m_is_detected){
        auto error = SetpointError();
        m_err_pub.publish(error);
    }
}

void Offboard::StateUpdate()
{ 
    if (m_prev_sm_state != m_sm_state){
        m_prev_sm_state = Ascii2Lower(m_sm_state);
    }
}


void Offboard::FlightRequest()
{
    m_offb_state = Enum2String(State::Flight);
}

geometry_msgs::Point Offboard::SetpointError()
{
    marker_point.x = 2.66001834608;
    marker_point.y = -2.29745275228;
    marker_point.z = 0.0;

    geometry_msgs::Point setpoint_point;
    setpoint_point.x = m_setpoint.pose.position.x;
    setpoint_point.y = m_setpoint.pose.position.y;
    setpoint_point.z = m_setpoint.pose.position.z;

    geometry_msgs::Point setpoint_error;
    setpoint_error.x = setpoint_point.x - marker_point.x;
    setpoint_error.y = setpoint_point.y - marker_point.y;
    setpoint_error.z = setpoint_point.z - marker_point.z;

    return setpoint_error;
}

void Offboard::SetpointnOffbStatePub() // Pulbish
{
    geometry_msgs::PoseStamped msg;
    msg.header = m_setpoint.header;
    msg.pose = m_setpoint.pose;
    m_local_pose_pub.publish(msg);
    // m_velocity_pub.publish(m_vel_setpoint);

    uav_msgs::OffboardState offboard_state_msg;
    offboard_state_msg.offb_mode = m_mavros_status.mode;
    offboard_state_msg.offb_state = m_offb_state;
    m_offboard_state_pub.publish(offboard_state_msg);
}