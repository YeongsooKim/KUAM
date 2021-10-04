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

#include "kuam_msgs/Setpoint.h"
#include "kuam_msgs/LandingState.h"
#include "kuam_msgs/LandRequest.h"
#include "kuam_state_machine/state_machine.h"
#include "kuam_payload_cmd/utils.h"

namespace kuam{
class Playload
{
private:
    // Node Handler
	ros::NodeHandle m_nh;
	ros::NodeHandle m_p_nh;
    Utils m_utils;

public:
    Playload();
    virtual ~Playload();

private:
    // Subscriber
    ros::Subscriber m_mavros_state_sub;
    ros::Subscriber m_setpoint_sub;
    ros::Subscriber m_trans_req_sub;
    ros::Subscriber m_command_sub;

    // Publisher
    ros::Publisher m_local_pos_tar_pub;
    ros::Publisher m_global_pose_pub;
    ros::Publisher m_payload_cmd_pub;
    
    // ServiceClient
    ros::ServiceClient m_arming_serv_client;
    ros::ServiceClient m_set_mode_serv_client;
    ros::ServiceServer m_landing_request_service;    

    ros::Timer m_timer;

    // Param
    string m_err_param;
    float m_process_freq_param;
    std::string m_data_ns_param;

    
    mavros_msgs::State m_mavros_status;
    kuam_msgs::Setpoint m_setpoint;
    ros::Time m_last_request_time;
    
	tf2_ros::Buffer m_tfBuffer;
	tf2_ros::TransformListener m_tfListener;

    std::string m_chat_cmd_mode;

private: // function
    // Initialization fucntion
    bool GetParam();
    bool InitFlag();
    bool InitROS();
    bool InitClient();
    
    // Process function
    void ProcessTimerCallback(const ros::TimerEvent& event);

    // Callback functions
    void ChatterCallback(const uav_msgs::Chat::ConstPtr &chat_ptr);
    bool LandRequestSrv(kuam_msgs::LandRequest::Request &req, kuam_msgs::LandRequest::Response &res);
    inline void MavrosStateCallback(const mavros_msgs::State::ConstPtr &state_ptr) { m_mavros_status = *state_ptr; }
    inline void SetpointCallback(const kuam_msgs::Setpoint::ConstPtr &setpoint_ptr) { m_setpoint = *setpoint_ptr; }

    // Request
    void ArmRequest();
    void ManualRequest();
    void AltitudeRequest();
    bool OffboardRequest();
    void LandRequest();
    void EmergRequest();

    // Util functions
    void StateUpdate();
    void SetpointPub();
    void PayloadCmdPub();
    bool IsMode(string input);
};
}
#endif //  __MAVROS_OFFB_H__