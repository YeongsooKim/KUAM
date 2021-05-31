#ifndef __MAVROS_OFFB_H__
#define __MAVROS_OFFB_H__

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geographic_msgs/GeoPath.h>
#include <geographic_msgs/GeoPose.h>
#include <geometry_msgs/Twist.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

#include <novatel_oem7_msgs/INSPVA.h>
#include <std_msgs/String.h>

#include "uav_msgs/Chat.h"
#include "uav_msgs/OffboardState.h"

#include "kuam_msgs/Setpoint.h"
#include "kuam_msgs/LandingState.h"
#include "kuam_mission_manager/state_machine.h"
#include "kuam_offb/utils.h"

namespace kuam{
class Offboard
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    Utils m_utils;

public:
    Offboard();
    virtual ~Offboard();

private:
    // Subscriber
    ros::Subscriber m_mavros_state_sub;
    ros::Subscriber m_sm_state_sub; // state machine state sub
    ros::Subscriber m_sm_mode_sub; // state machine mode sub
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_geo_setpoint_sub;
    ros::Subscriber m_target_waypoints_sub;
    ros::Subscriber m_landing_state_sub;

    ros::Subscriber m_current_global_pose_sub;
    ros::Subscriber m_command_sub;

    // Publisher
    ros::Publisher m_local_pose_pub;
    ros::Publisher m_global_pose_pub;
    ros::Publisher m_velocity_pub;
    ros::Publisher m_offboard_state_pub;
    ros::Publisher m_err_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;

    ros::Timer m_timer;

    // Param
    float m_setpoint_pub_interval_param;
    std::string m_maneuver_ns_param;
    bool m_is_debug_mode_param;

    // Flag
    bool m_is_global;
    bool m_offb_init;
    bool m_is_detected;
    bool m_is_land;
    
    ros::Time m_last_request_time;
    mavros_msgs::State m_mavros_status;
    std::string m_sm_state;
    std::string m_prev_sm_state;
    std::string m_sm_mode;
    std::string m_offb_state;
    kuam_msgs::Setpoint m_setpoint;
    geographic_msgs::GeoPoseStamped m_global_setpoint;
    geometry_msgs::Twist m_vel_setpoint;
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;
    unsigned int m_hovering_count;

private: // function
    // Initialization fucntion
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitClient();
    
    // Process function
    void ProcessTimerCallback(const ros::TimerEvent& event);

    // Callback functions
    inline void MavrosStateCallback(const mavros_msgs::State::ConstPtr &state_ptr) { m_mavros_status = *state_ptr; }
    inline void SMStateCallback(const std_msgs::String::ConstPtr &state_ptr) { m_sm_state = state_ptr->data; }
    inline void SMModeCallback(const std_msgs::String::ConstPtr &mode_ptr) { m_sm_mode = mode_ptr->data; }
    inline void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr) { m_setpoint = *setpoint_ptr; }
    inline void LandingStateCallback(const kuam_msgs::LandingState::ConstPtr &state_ptr) { m_is_detected = state_ptr->is_detected; m_is_land = state_ptr->is_land; }

    // Request
    void ManualRequest();
    void ArmRequest();
    void HoveringRequest();
    void TakeoffRequest();
    void LandingRequest();
    void FlightRequest();

    // Util functions
    bool IsOffboard();
    geometry_msgs::Point SetpointError();
    void StateUpdate();
    void SetpointnOffbStatePub();
};
}
#endif //  __MAVROS_OFFB_H__