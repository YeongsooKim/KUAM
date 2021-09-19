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

#include <std_msgs/String.h>

#include "uav_msgs/Chat.h"
#include "uav_msgs/PayloadCmd.h"

#include "kuam_msgs/TransReq.h"
#include "kuam_msgs/Setpoint.h"
#include "kuam_msgs/LandingState.h"
#include "kuam_mission_manager/state_machine.h"
#include "kuam_payload_cmd/utils.h"

namespace kuam{
class Playload
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
    Utils m_utils;

public:
    Playload();
    virtual ~Playload();

private:
    // Subscriber
    ros::Subscriber m_mavros_state_sub;
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_trans_req_sub;

    // Publisher
    ros::Publisher m_local_pos_tar_pub;
    ros::Publisher m_global_pose_pub;
    ros::Publisher m_payload_cmd_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;

    ros::Timer m_timer;

    // Param
    float m_process_freq_param;
    std::string m_maneuver_ns_param;
    bool m_is_debug_mode_param;

    // Flag
    
    ros::Time m_last_request_time;

    mavros_msgs::State m_mavros_status;
    
    std::string m_cur_sm_state;
    std::string m_prev_sm_state;
    std::string m_sm_kuam_mode; // state machine kuam mode
    std::string m_px4_mode;
    std::string m_payload_cmd_state;

    kuam_msgs::Setpoint m_setpoint;
    
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

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
    inline void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr) { m_setpoint = *setpoint_ptr; }
    inline void TransReqCallback(const kuam_msgs::TransReq::ConstPtr &trans_req_ptr) { 
        m_sm_kuam_mode = trans_req_ptr->mode.kuam;
        m_px4_mode = trans_req_ptr->mode.px4;
        m_cur_sm_state = trans_req_ptr->state; 
    }
    

    // Request
    void ManualRequest();
    void AltitudeRequest();
    void EmergRequest();
    void ArmRequest();
    void HoveringRequest();
    void TakeoffRequest();
    void LandingRequest();
    void FlightRequest();

    // Util functions
    bool IsOffboard();
    void StateUpdate();
    void SetpointPub();
    void PayloadCmdPub();

};
}
#endif //  __MAVROS_OFFB_H__