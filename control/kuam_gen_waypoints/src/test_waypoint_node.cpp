#include "ros/ros.h"
#include <ros/spinner.h>
#include <cstdlib>

#include "uav_msgs/TargetState.h"
#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>

#include <tf/tf.h>

using namespace std;

class TestWaypoint{
public:
    TestWaypoint();
    ~TestWaypoint();

private:
    ros::NodeHandle m_nh;

    // Initialize publisher
    ros::Publisher m_target_state_pub;

    // Initialize timer
    ros::Timer m_test_waypoint_timer;

    uav_msgs::TargetState m_target_state;

    bool GetParam();
    void InitROS();

    void TestWaypointTimerCallback(const ros::TimerEvent& event);
    
public:
    void SetTestWaypoint(geometry_msgs::Pose pose);
};

TestWaypoint::TestWaypoint()
{
    InitROS();
}

TestWaypoint::~TestWaypoint()
{}

void TestWaypoint::InitROS()
{
    // package, node, topic name
    string node_name_with_namespace = ros::this_node::getName();

    // Initialize publisher
    m_target_state_pub = m_nh.advertise<uav_msgs::TargetState>(node_name_with_namespace + "/waypoints", 10);

    // Initialize timer
    m_test_waypoint_timer = m_nh.createTimer(ros::Duration(0.1), &TestWaypoint::TestWaypointTimerCallback, this);
}


void TestWaypoint::TestWaypointTimerCallback(const ros::TimerEvent& event)
{
    uav_msgs::TargetState target_state;
    target_state.pose = m_target_state.pose;

    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    target_state.header = header;

    m_target_state_pub.publish(target_state);
}

void TestWaypoint::SetTestWaypoint(geometry_msgs::Pose pose)
{
    m_target_state.pose = pose;
}

int main(int argc, char ** argv)
{
    // Initialize ROS
	ros::init (argc, argv, "test_waypoint_node");
    TestWaypoint test_waypoint;

    geometry_msgs::Pose pose;
    pose.position.x = atof(argv[1]);
    pose.position.y = atof(argv[2]);
    pose.position.z = atof(argv[3]);

    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();
    
    test_waypoint.SetTestWaypoint(pose);

    ros::spin();
    return 0;
}